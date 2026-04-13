import torch
import torch.nn as nn
import torch.nn.functional as F
import torch.optim as optim
from tqdm import tqdm
import matplotlib.pyplot as plt
import statistics
import numpy as np
from torch.utils.data import TensorDataset, DataLoader
import torch.multiprocessing as mp
from concurrent.futures import ThreadPoolExecutor, ProcessPoolExecutor
import time
from torch.nn.utils.rnn import pad_packed_sequence

'''
Optimized Hyperparameter settings for better GPU utilization
'''
WINDOW_SIZE = 180
INPUT_SIZE = 22 ## To change here to 19
HIDDEN_DIM = 64
LR = 0.003
EPOCH = 100 #Instead of 500 for test
BATCH_SIZE = 128  # Increased for better GPU utilization
NUM_WORKERS = 32  # For DataLoader parallelization
PREFETCH_FACTOR = 8  # For DataLoader optimization
HANDOFF_ITERATIONS = 30

# Device configuration
device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
print(f"Using device: {device}")
if device.type == 'cuda':
    print(f"GPU: {torch.cuda.get_device_name(0)}")
    print(f"GPU Memory: {torch.cuda.get_device_properties(0).total_memory / 1e9:.1f} GB")

class LSTM_wind_estimator(nn.Module):
    def __init__(self, hidden_dim, input_size, num_layers=4, dropout=0.1):
        super(LSTM_wind_estimator, self).__init__()
        self.hidden_dim = hidden_dim
        self.input_size = input_size
        self.num_layers = num_layers
        
        # Enhanced LSTM with multiple layers and dropout
        self.lstm = nn.LSTM(
            input_size=input_size, 
            hidden_size=hidden_dim, 
            num_layers=num_layers,
            batch_first=True,
            dropout=dropout if num_layers > 1 else 0
        )
        self.dropout = nn.Dropout(dropout)
        self.linear = nn.Linear(in_features=hidden_dim, out_features=3)

    def forward(self, window):
        # window shape: (batch_size, sequence_length, input_size)
        lstm_out, _ = self.lstm(window)
        # Apply dropout before final layer
        lstm_out = self.dropout(lstm_out)
        # Only take the last timestep output
        velocity_space = self.linear(lstm_out[:, -1, :])
        return velocity_space

def create_sliding_windows_vectorized(data, window_size):
    """
    Vectorized sliding window creation using unfold operation
    Much faster than the original loop-based approach
    """
    # data shape: (sequence_length, features)
    sequence_length, num_features = data.shape
    
    if sequence_length < window_size:
        raise ValueError(f"Sequence length {sequence_length} is less than window size {window_size}")
    
    # Use unfold to create sliding windows efficiently
    # This creates a view of the data without copying
    windows = data.unfold(0, window_size, 1)  # (num_windows, features, window_size)
    windows = windows.permute(0, 2, 1)  # (num_windows, window_size, features)
    
    return windows

def generate_training_batch(observations, infos, device):

    inputs = []
    targets = []

    padded_observations, lengths_observations = pad_packed_sequence(observations, batch_first=True)
    padded_info, lengths_infos = pad_packed_sequence(infos, batch_first=True)

    observations = padded_observations
    infos = padded_info
    for env_idx, observation in enumerate(padded_observations):
        max_start = lengths_observations[env_idx] - WINDOW_SIZE - 2     # last starting index
        for t in range(max_start):
            x = observations[env_idx, t:t+WINDOW_SIZE]                  # (seq_len, obs_dim)
            y = infos[env_idx, t+WINDOW_SIZE+1, :]                      # (target_dim,)
            
            inputs.append(x)
            targets.append(y)
            
    return torch.stack(inputs, dim=0), torch.stack(targets, dim=0)

def prepare_batch_data(episodes):
    """
    Prepare batched training data from multiple episodes
    """
    all_windows = []
    all_targets = []
    
    for training_data, wind_along in episodes:
        if len(training_data) <= WINDOW_SIZE:
            continue  # Skip episodes that are too short
            
        # Create sliding windows using vectorized approach
        windows = create_sliding_windows_vectorized(training_data, WINDOW_SIZE)
        targets = wind_along[WINDOW_SIZE:]
        
        # Ensure targets match windows
        min_len = min(len(windows), len(targets))
        windows = windows[:min_len]
        targets = targets[:min_len]
        
        all_windows.append(windows)
        all_targets.append(targets)
    
    if not all_windows:
        return None, None
    
    # Concatenate all windows and targets
    batch_windows = torch.cat(all_windows, dim=0)
    batch_targets = torch.cat(all_targets, dim=0)
    
    return batch_windows, batch_targets

def train_optimized(observations, infos, model, device):
    """
    Optimized training function with vectorization and parallelization
    """
    loss_function = nn.MSELoss()
    optimizer = optim.Adam(model.parameters(), lr=LR, weight_decay=1e-5)  # Adam often works better than SGD
    scheduler = optim.lr_scheduler.ReduceLROnPlateau(optimizer, 'min', patience=10, factor=0.5)
    
    history = []
    best_loss = float('inf')

    # Enable mixed precision training for faster computation on modern GPUs
    # scaler = torch.amp.GradScaler('cuda') if device.type == 'cuda' else None
    scaler = torch.amp.GradScaler(device=device)
    
    print("Starting wind_estimator training...")
    start_time = time.time()
    
    # Generate batch of episodes - use fewer episodes but larger batches for efficiency
    batch_windows, batch_targets = generate_training_batch(observations=observations, infos=infos, device=device)
    
    if batch_windows is None:
        return
    # Create DataLoader for efficient batching
    dataset = TensorDataset(batch_windows.cpu(), batch_targets.cpu())
    
    loader = DataLoader(
        dataset, 
        batch_size=BATCH_SIZE, 
        shuffle=True, 
        num_workers=4,  # Set to 0 for CUDA tensors
        pin_memory=True  # Disable pin_memory for GPU tensors
    )
    
    epoch_loss = 0
    num_batches = 0
    
    for epoch in tqdm(range(EPOCH), desc="Training Progress"):
        model.train()
        epoch_start = time.time()
        
        for batch_x, batch_y in loader:
            
            batch_x = batch_x.to(device, non_blocking=True)
            batch_y = batch_y.to(device, non_blocking=True)
            optimizer.zero_grad()
            
            if scaler is not None:
                # Mixed precision training
                with torch.amp.autocast('cuda'):
                    output = model(batch_x)
                    loss = loss_function(output.squeeze(), batch_y)
                
                scaler.scale(loss).backward()
                scaler.step(optimizer)
                scaler.update()
            else:
                # Standard training
                output = model(batch_x)
                loss = loss_function(output.squeeze(), batch_y)
                loss.backward()
                optimizer.step()
            
            epoch_loss += loss.item()
            num_batches += 1
        
        avg_epoch_loss = epoch_loss / max(num_batches, 1)
        history.append(avg_epoch_loss)
        
        # Learning rate scheduling
        scheduler.step(avg_epoch_loss)
        
        # Save best model
        if avg_epoch_loss < best_loss:
            best_loss = avg_epoch_loss
            torch.save(model.state_dict(), './wind_estimator_best.mdl')
        
        epoch_time = time.time() - epoch_start
        # if epoch % 10 == 0:
        #     print(f"Epoch {epoch}: Loss = {avg_epoch_loss:.6f}, Time = {epoch_time:.2f}s")
    
    total_time = time.time() - start_time
    print(f"Training completed in {total_time:.2f} seconds")
    
    # Plot training history
    # plt.figure(figsize=(10, 6))
    # plt.plot(range(len(history)), history)
    # plt.title("Training Loss Over Epochs (Optimized)")
    # plt.xlabel("Epoch")
    # plt.ylabel("Loss")
    # plt.yscale('log')
    # plt.grid(True)
    # plt.show()

    return model

def predict(observations, model, device):
    num_sequences = observations.shape[0]

    outputs = torch.zeros(num_sequences, 3, device=device)
    # Loop through batches
    for start in range(0, num_sequences, BATCH_SIZE):
        end = start + BATCH_SIZE
        batch = observations[start:end]  # shape: (batch_size_i, 180, 22)
        out = model(batch)
        # Assign directly to the correct slice in outputs
        if out.shape[0] == BATCH_SIZE:
            outputs[start:end] = out
        else:
            outputs[start:start + out.shape[0]] = out
    if outputs.shape[0] < BATCH_SIZE:
        return outputs.squeeze(0)
    return outputs

def predict_cpu(observations, model):
    observations = torch.tensor(observations)

    observations = observations.unsqueeze(0)

    # Shape of np array is now 1,180,22

    out = model(observations)

    return out.squeeze(0)

def eval_wind_optimized():
    """
    Optimized evaluation with batch processing
    """
    model = LSTM_wind_estimator(hidden_dim=HIDDEN_DIM, input_size=INPUT_SIZE).to(device)
    model.load_state_dict(torch.load('wind_estimator_best_most_recent.mdl', map_location=device))
    model.eval()

    print("Generating test data...")
    distance, wind, bearing, v_x, v_y, omega, pitch, action_left, action_right = eval(render=False, p_ground_truth = 1)
    
    # Prepare test data on CPU first, then move to GPU
    training_data = torch.stack([
        torch.tensor(distance, dtype=torch.float32),
        torch.tensor(bearing, dtype=torch.float32),
        torch.tensor(omega, dtype=torch.float32),
        torch.tensor(pitch, dtype=torch.float32),
        torch.tensor(np.array(v_x), dtype=torch.float32),
        torch.tensor(np.array(v_y), dtype=torch.float32),
        torch.tensor(action_left, dtype=torch.float32),
        torch.tensor(action_right, dtype=torch.float32)
    ], dim=-1).to(device)
    
    wind_along = wind[:, 0]
    wind_along_target = wind_along[WINDOW_SIZE:]

    # Create sliding windows vectorized
    testing_windows = create_sliding_windows_vectorized(training_data, WINDOW_SIZE)
    
    # Ensure target length matches windows
    min_len = min(len(testing_windows), len(wind_along_target))
    testing_windows = testing_windows[:min_len]
    wind_along_target = wind_along_target[:min_len]

    print("Running batch inference...")
    with torch.no_grad():
        # Batch inference instead of sequential
        batch_size = 1  # Process multiple windows at once
        wind_along_preds = []

        '''Low pass filtering'''
        PASS_FILTER_SIZE = 3
        sliding_window = []

        for i in range(0, len(testing_windows), batch_size):
            batch = testing_windows[i:i+batch_size]
            prediction = model(batch).cpu().numpy().flatten()
            if (len(sliding_window) > PASS_FILTER_SIZE and PASS_FILTER_SIZE != 0):
                prediction  = (sum(sliding_window[-PASS_FILTER_SIZE:]) + prediction)/(PASS_FILTER_SIZE + 1)
            sliding_window.append(prediction)
            wind_along_preds.extend(prediction)
    
    # Convert to numpy for plotting
    wind_along_preds = np.array(wind_along_preds)
    wind_along_target = np.array(wind_along_target)
    
    # Calculate error
    error_along = wind_along_preds - wind_along_target
    
    # Plotting
    plt.figure(figsize=(15, 10))
    
    plt.subplot(2, 1, 1)
    plt.plot(range(len(wind_along_preds)), wind_along_preds, color="black", alpha=0.7, label="Estimated wind", linewidth=1)
    plt.plot(range(len(wind_along_target)), wind_along_target, color="red", alpha=0.7, label="True wind", linewidth=1)
    plt.ylabel("Wind velocity magnitude (m/s)")
    plt.xlabel("Time (s)")
    plt.title("Wind Estimator Performance (Optimized)")
    plt.legend()
    plt.grid(True, alpha=0.3)

    plt.subplot(2, 1, 2)
    plt.plot(range(len(error_along)), error_along, color="black", alpha=0.7, linewidth=1)
    plt.ylabel("Error (m/s)")
    plt.xlabel("Time (s)")
    plt.title(f"Wind Estimator Error - Mean: {np.mean(error_along):.4f}, Std: {np.std(error_along):.4f}")
    plt.grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.show()
    
    print(f"Mean Absolute Error: {np.mean(np.abs(error_along)):.4f}")
    print(f"Root Mean Square Error: {np.sqrt(np.mean(error_along**2)):.4f}")

def load_wind_estimator_optimized():
    """
    Load the optimized wind estimator model
    """
    model = LSTM_wind_estimator(hidden_dim=HIDDEN_DIM, input_size=INPUT_SIZE).to(device)
    model.load_state_dict(torch.load('./wind_estimator_best_most_recent.mdl', map_location=device))
    model.eval()
    return model, WINDOW_SIZE

def benchmark_comparison():
    """
    Compare performance between original and optimized versions
    """
    print("=== Performance Benchmark ===")
    
    # Test data generation time
    print("\n1. Testing data generation...")
    start_time = time.time()
    episodes = generate_training_batch(batch_size=8)
    batch_windows, batch_targets = prepare_batch_data(episodes)
    data_gen_time = time.time() - start_time
    print(f"Vectorized data generation: {data_gen_time:.2f}s")
    print(f"Generated {len(batch_windows)} training samples")
    
    # Test model inference speed
    print("\n2. Testing model inference...")
    model = LSTM_wind_estimator(hidden_dim=HIDDEN_DIM, input_size=INPUT_SIZE).to(device)
    model.eval()
    
    # Move benchmark data to device
    batch_windows = batch_windows.to(device)
    batch_targets = batch_targets.to(device)
    
    # Single sample inference (original method)
    single_sample = batch_windows[:1]
    start_time = time.time()
    with torch.no_grad():
        for _ in range(100):
            _ = model(single_sample)
    single_inference_time = time.time() - start_time
    
    # Batch inference (optimized method)
    batch_sample = batch_windows[:64] if len(batch_windows) >= 64 else batch_windows
    start_time = time.time()
    with torch.no_grad():
        _ = model(batch_sample)
    batch_inference_time = time.time() - start_time
    
    print(f"Single inference (100 samples): {single_inference_time:.4f}s")
    print(f"Batch inference ({len(batch_sample)} samples): {batch_inference_time:.4f}s")
    print(f"Speedup factor: {single_inference_time/batch_inference_time:.2f}x")


if __name__ == "__main__":
    print("=== Optimized Wind Estimator ===")
    
    # # Run benchmark
    # benchmark_comparison()
    
    # print("\nStarting optimized training and evaluation...")
    
    # # Train optimized model

    # for it in range(HANDOFF_ITERATIONS):
    #     """
    #     Calculate the probability for the wind_estimator to be used 
    #     """
    #     p_ground_truth = 1 - it/HANDOFF_ITERATIONS
    #     model = train_optimized(p_ground_truth=p_ground_truth)
    #     train(p_ground_truth=p_ground_truth)
    
    # Evaluate optimized model
    # eval_wind_optimized()