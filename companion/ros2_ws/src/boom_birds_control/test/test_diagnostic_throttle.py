import pytest
from boom_birds_control.diagnostic_throttle import DiagnosticThrottle


def test_period_and_immediate_fault_and_clock_reset():
    gate = DiagnosticThrottle(5.)
    assert gate.due(10., ("OK",))
    assert not gate.due(10.02, ("OK",))
    assert gate.due(10.03, ("FAULT",))
    assert not gate.due(10.1, ("FAULT",))
    assert gate.due(10.24, ("FAULT",))
    assert gate.due(9., ("FAULT",))


@pytest.mark.parametrize("rate", [0., -1., float("nan"), float("inf")])
def test_invalid_rate(rate):
    with pytest.raises(ValueError):
        DiagnosticThrottle(rate)


def test_control_feedback_still_published_each_tick():
    from types import SimpleNamespace
    import rclpy
    from rclpy.context import Context
    from rclpy.parameter import Parameter
    from boom_birds_control.px4_interface_node import Px4InterfaceNode
    context = Context()
    rclpy.init(context=context)
    node = Px4InterfaceNode(context=context, parameter_overrides=[Parameter("require_session", value=True)])
    execution, diagnostics = [], []
    node.pub_execution = SimpleNamespace(publish=execution.append)
    node.pub_status = SimpleNamespace(publish=diagnostics.append)
    try:
        for _ in range(50):
            node._tick()
        assert len(execution) == 50
        assert len(diagnostics) < len(execution)
        assert all(not message.sending for message in execution)
    finally:
        node.destroy_node()
        context.shutdown()
