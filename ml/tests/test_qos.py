import pytest

from nuway_ml.common.qos import QOS

pytestmark = pytest.mark.import_light


def test_qos_table_matches_docs_02_section_3_11():
    assert set(QOS) == {"sensor", "stream", "latched", "event", "diag", "viz"}
    assert QOS["stream"].reliability == "reliable"
    assert QOS["stream"].depth == 2
    assert QOS["latched"].durability == "transient_local"
    assert QOS["event"].depth == 10
    assert QOS["sensor"].reliability == "best_effort"
