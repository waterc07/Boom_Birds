import importlib.util
import struct
from pathlib import Path
import pytest

spec = importlib.util.spec_from_file_location('bag_header_timing', Path(__file__).parents[1]/'bag_header_timing.py')
timing = importlib.util.module_from_spec(spec)
spec.loader.exec_module(timing)


@pytest.mark.parametrize('endian,enc', [('<',b'\x00\x01'),('>',b'\x00\x00')])
def test_cdr_header(endian,enc):
    assert timing.headerNs(enc+b'\x00\x00'+struct.pack(endian+'iI',123,456)) == 123_000_000_456


@pytest.mark.parametrize('prefix', [b'', b'\x00\x03'+b'\x00'*10,
    b'\x00\x01\x00\x00'+struct.pack('<iI',1,1_000_000_000),
    b'\x00\x01\x00\x00'+struct.pack('<iI',-1,0), b'\x00'*12])
def test_invalid_header(prefix):
    with pytest.raises(ValueError): timing.headerNs(prefix)


def test_receive_pause_and_true_source_gap_are_distinct():
    batching = timing.timingStats([(1_000_000_000,1_000_000_000),(1_400_000_000,1_020_000_000)])
    assert batching['receive']['max_forward_gap_s'] == .4
    assert batching['header']['max_forward_gap_s'] == .02
    source = timing.timingStats([(1_000_000_000,1_000_000_000),(1_400_000_000,1_400_000_000)])
    assert source['header']['max_forward_gap_s'] == .4


def test_out_of_order_is_not_sorted_away():
    result = timing.timingStats([(2,3),(4,2),(5,2)])
    assert result['header']['backward_steps'] == 1
    assert result['header']['duplicate_steps'] == 1


def test_read_only_sqlite_window_and_invalid_header(tmp_path):
    import json
    import sqlite3
    db = tmp_path/'bag.db3'
    events = tmp_path/'events.json'
    events.write_text(json.dumps([{'phase':'STILL','capture_received_ns':30_000_000_000}]))
    with sqlite3.connect(db) as con:
        con.execute('create table topics(id integer,name text,type text,serialization_format text)')
        con.execute('create table messages(id integer primary key,topic_id integer,timestamp integer,data blob)')
        for tid,(name,kind) in enumerate(timing.RAW_TYPES.items(),1):
            con.execute('insert into topics values(?,?,?,?)',(tid,name,kind,'cdr'))
            for sec in range(9,42):
                con.execute('insert into messages(topic_id,timestamp,data) values(?,?,?)',
                    (tid,sec*1_000_000_000,b'\x00\x01\x00\x00'+struct.pack('<iI',sec,0)))
        con.execute('insert into messages(topic_id,timestamp,data) values(1,42000000000,?)',(b'bad',))
    original = db.read_bytes()
    report = timing.analyze(db,events)
    assert db.read_bytes() == original
    assert not report['changes_original_window_verdict']
    first = report['topics'][next(iter(timing.RAW_TYPES))]
    assert first['receive_window_covered']
    assert first['invalid_headers'] == 1
    assert first['receive_window_with_bracketing_samples']['messages'] == 33
    with sqlite3.connect(db) as con:
        con.execute("update topics set type='std_msgs/msg/String' where id=1")
    with pytest.raises(ValueError,match='unsupported type'):
        timing.analyze(db,events)
