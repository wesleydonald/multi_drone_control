"""A tracker disarmed alone at its rest keeps its process up (one-shot): a tracker exiting mid-flight stalled the
graph ~0.5 s and two carriers timed out on pose (R1378, reviewer 9 Oct)."""
import types

from utility_objects.callback_manager_multi import CallbackManagerMulti


def _cb(keep):
    node = types.SimpleNamespace(armed=True, takeoff_requested=True, shutdown_requested=False,
                                 keep_alive_on_disarm=keep,
                                 get_logger=lambda: types.SimpleNamespace(info=lambda *a: None, warn=lambda *a: None))
    cb = types.SimpleNamespace(node=node, drone_id=3, publish_arming_state=lambda: None)
    return cb, node


def _disarm(cb):
    return CallbackManagerMulti.handle_arming_service(cb, types.SimpleNamespace(arm=False),
                                                      types.SimpleNamespace(success=None, message=''))


def test_a_disarm_alone_keeps_the_controller_up_once():
    cb, node = _cb(keep=True)
    r = _disarm(cb)
    assert r.success and not node.armed and not node.shutdown_requested
    assert node.keep_alive_on_disarm is False             # one-shot
    node.armed = True                                      # a later disarm (the fleet's, after LANDED)
    _disarm(cb)
    assert node.shutdown_requested                         # flown: closes as before


def test_an_ordinary_disarm_after_a_flight_still_shuts_down():
    cb, node = _cb(keep=False)
    _disarm(cb)
    assert node.shutdown_requested
