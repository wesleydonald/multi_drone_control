"""Lean rclpy (switch, default off): no default QoS event handlers on publishers and subscriptions.

Loaded at interpreter start only with this folder on PYTHONPATH and MDC_LEAN_RCLPY=1. Humble's executor
re-adds every handler to its wait set on each wake-up; a tracker-shaped node spent 16 % less CPU without
them (docs/analysis/rig_load_2026-10-14.md). Costs only the "incompatible QoS" warnings. A node that
passes its own event_callbacks keeps them.
"""
import os

if os.environ.get('MDC_LEAN_RCLPY') == '1':
    try:
        import rclpy.node as _node
        from rclpy.qos_event import PublisherEventCallbacks, SubscriptionEventCallbacks

        _create_publisher = _node.Node.create_publisher
        _create_subscription = _node.Node.create_subscription

        def create_publisher(self, *args, event_callbacks=None, **kwargs):
            return _create_publisher(self, *args, event_callbacks=event_callbacks
                                     or PublisherEventCallbacks(use_default_callbacks=False), **kwargs)

        def create_subscription(self, *args, event_callbacks=None, **kwargs):
            return _create_subscription(self, *args, event_callbacks=event_callbacks
                                        or SubscriptionEventCallbacks(use_default_callbacks=False), **kwargs)

        _node.Node.create_publisher = create_publisher
        _node.Node.create_subscription = create_subscription
    except Exception:       # no rclpy in this interpreter: nothing to patch
        pass
