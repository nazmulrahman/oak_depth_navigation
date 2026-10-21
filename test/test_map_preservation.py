"""Check map preservation independently of camera hardware."""
from pathlib import Path
import pytest
from oak_depth_navigation.bringup import session_database


def test_existing_mapping_database_is_rejected(tmp_path):
    db = tmp_path / 'precious.db'
    db.write_bytes(b'original')
    with pytest.raises(RuntimeError, match='Refusing to overwrite'):
        session_database(str(db))
    assert db.read_bytes() == b'original'


def test_localization_copies_source(tmp_path, monkeypatch):
    monkeypatch.setattr(Path, 'home', lambda: tmp_path)
    db = tmp_path / 'saved.db'
    db.write_bytes(b'saved map')
    copy = Path(session_database(str(db), localization=True))
    assert copy != db
    assert copy.read_bytes() == b'saved map'
    copy.write_bytes(b'localization changes')
    assert db.read_bytes() == b'saved map'


def test_default_sessions_are_unique(tmp_path, monkeypatch):
    monkeypatch.setattr(Path, 'home', lambda: tmp_path)
    one = session_database()
    two = session_database()
    assert one != two


def test_missing_localization_database_is_rejected(tmp_path):
    with pytest.raises(RuntimeError, match='localization requires'):
        session_database(str(tmp_path / 'absent.db'), localization=True)
