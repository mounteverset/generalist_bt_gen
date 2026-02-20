from user_interface.command_router import parse_command, parse_session_and_note


def test_parse_command_splits_payload():
    parsed = parse_command('/approve session-123 looks good')
    assert parsed.name == 'approve'
    assert parsed.payload == 'session-123 looks good'


def test_parse_session_and_note_with_session_id():
    session_id, note = parse_session_and_note('chatui-20260101T000000Z-1 approved')
    assert session_id == 'chatui-20260101T000000Z-1'
    assert note == 'approved'


def test_parse_session_and_note_without_session_id():
    session_id, note = parse_session_and_note('not yet, adjust payload')
    assert session_id == ''
    assert note == 'not yet, adjust payload'
