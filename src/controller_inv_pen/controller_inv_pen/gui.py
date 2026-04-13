import pygame
import re
import numpy as np
class GUI:
    def __init__(self, controller):
        self.controller = controller
        pygame.init()
        self.screen = pygame.display.set_mode((800, 600))
        self.font = pygame.font.Font(None, 36)
        
        # Arm/Disarm button
        self.button_rect = pygame.Rect(100, 100, 150, 50)
        self.button_text = "Arm"
        self.button_color = (0, 255, 0)

        # Enable FIP controller button (default is navigation control)
        self.button_FIP = pygame.Rect(300, 100, 150, 50)
        self.button_FIP_text = "FIP"
        self.button_FIP_color = (0, 255, 0)

        # Land button
        self.button_land = pygame.Rect(500, 100, 150, 50)
        self.button_land_text = "Land"
        self.button_land_color = (0, 255, 0)

        # Set point input 
        self.setpoint_button = pygame.Rect(300, 250, 150, 50)
        self.setpoint_text = "x,y,z"
        self.setpoint_active_color = (255, 0, 0)
        self.setpoint_inactive_color = (0, 255, 0)
        self.setpoint_color = self.setpoint_inactive_color
        self.setpoint_active = False


    def handle_events(self):
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                self.controller.on_close()
            elif event.type == pygame.MOUSEBUTTONDOWN:
                # Arm/Disarm button
                if self.button_rect.collidepoint(event.pos):
                    self.controller.get_logger().info(f'Button press happened state {self.controller.armed}')
                    if not self.controller.armed:
                        self.controller.armed = True
                        self.update_button("Disarm", (255, 0, 0))
                    else: 
                        self.controller.armed = False
                        self.update_button("Arm", (0, 255, 0))
                
                # FIP button 
                if self.button_FIP.collidepoint(event.pos):
                    if not self.controller.useFIP:
                        self.controller.useFIP = True
                        self.update_FIP_button("Nav", (255, 0, 0))
                    else: 
                        self.controller.useFIP = False
                        self.update_FIP_button("FIP", (0, 0, 255))

                # land button 
                if self.button_land.collidepoint(event.pos):
                    if not self.controller.land:
                        self.controller.land = True
                   
                # setpoint 
                if self.setpoint_button.collidepoint(event.pos):
                    self.setpoint_active = not self.setpoint_active
                else:
                    self.setpoint_active = False 
                    

            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_SPACE: 
                    self.controller.armed = False 
                    self.update_button("Arm", (0, 255, 0))

                # setpoint stuff 
                if self.setpoint_active:
                    
                    if event.key == pygame.K_RETURN: # Reset setpoint text
                        print(self.setpoint_text)
                        # send setpoint to controller 
                        
                        dataToParse = self.setpoint_text
                        setpoint = []
                        valid = True
                        if dataToParse != "" and re.sub(",", "", dataToParse) != "":
                            dataToParse = re.sub("\s", "", dataToParse)
                            dataSegments = re.split(",", dataToParse)
                            for data in dataSegments:
                                try:
                                    val = float(data)
                                    setpoint.append(val)
                                except Exception as e:
                                    valid = False
                                    print(f"Error: cannot convert {data} to float")
                               
                            if valid and len(setpoint) != 3:
                                valid = False
                                print(f"Error: setpoint length was {len(setpoint)} but expected length is 3")
                          
                                
                            if valid:
                                try:
                                    setpoint = np.array(setpoint)
                                    if -3.0 <= setpoint[0] and setpoint[0] <= 3.0 and -3.0 <= setpoint[1]  and setpoint[1] <= 3.0 and 0.0 <= setpoint[2] and setpoint[2] <= 3.0:
                                        self.controller.setpoint = setpoint
                                        print(f"Setpoint changed to x: {setpoint[0]}, y: {setpoint[1]}, z: {setpoint[2]}")
                                    else:
                                        print(f"Invalid ranges, expected ranges are -3 <= x <= 3, -3 <= y <= 3, 0 <= z <= 3 ")
                                except Exception as e:
                                    print(f"Caught exception: {e}")

                        self.setpoint_text = "x,y,z"


                                

                    elif event.key == pygame.K_BACKSPACE: # Delete previous character 
                        self.setpoint_text = self.setpoint_text[:-1]
                    else:
                        self.setpoint_text += event.unicode

            if self.setpoint_active:
                self.setpoint_color = self.setpoint_active_color
            else: 
                self.setpoint_color = self.setpoint_inactive_color

        self.screen.fill((0, 0, 0))
        
        # Draw Arm/Disarm button
        pygame.draw.rect(self.screen, self.button_color, self.button_rect)
        text_surface = self.font.render(self.button_text, True, (255, 255, 255))
        text_rect = text_surface.get_rect(center=self.button_rect.center)
        self.screen.blit(text_surface, text_rect)

        # Draw FIP button
        pygame.draw.rect(self.screen, self.button_FIP_color, self.button_FIP)
        FIP_text_surface = self.font.render(self.button_FIP_text, True, (255, 255, 255))
        FIP_text_rect = FIP_text_surface.get_rect(center=self.button_FIP.center)
        self.screen.blit(FIP_text_surface, FIP_text_rect)

        # Draw land button
        pygame.draw.rect(self.screen, self.button_land_color, self.button_land)
        land_text_surface = self.font.render(self.button_land_text, True, (255, 255, 255))
        land_text_rect = land_text_surface.get_rect(center=self.button_land.center)
        self.screen.blit(land_text_surface, land_text_rect)

        # Draw setpoint
        pygame.draw.rect(self.screen, self.setpoint_color, self.setpoint_button)
        setpoint_text_surface = self.font.render(self.setpoint_text, True, (255, 255, 255))
        setpoint_text_rect = setpoint_text_surface.get_rect(center=self.setpoint_button.center)
        self.screen.blit(setpoint_text_surface, setpoint_text_rect)


        pygame.display.flip()

    def update_button(self, text, color):
        self.button_text = text
        self.button_color = color

    def update_FIP_button(self, text, color):
        self.button_FIP_text = text
        self.button_FIP_color = color

    '''def update_land_button(self, text, color):
        self.button_land_text = text
        self.button_land_color = color'''

    def quit(self):
        pygame.quit()
