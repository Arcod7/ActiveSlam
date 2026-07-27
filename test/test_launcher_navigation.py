"""A list longer than its pane must say so, and must not wrap under a held key.

Two failures the option list had. Rows past the bottom of the pane were drawn
identically to a list that simply ends there, so options existed with nothing
on screen saying they did. And the cursor wrapped the instant it reached an
end, so holding Down to reach the last option shot past it and back to the top
— the list has no fixed length (the advanced set and the section folds change
it), which makes counting keypresses no way to arrive anywhere.

These pin the cut marks appearing only on a cut side, and the ends holding
under auto-repeat while a deliberate second press still wraps. Holding is
recognised from the gap between presses *and* from repeats found queued behind
one: the control screen spins ROS and repaints between reads, and on a frame
slower than the gap every repeat would otherwise read as a fresh press and wrap
the list anyway — which is how the first attempt at this failed on the real
screen while passing a timing-only test.
"""
import os
import sys
import time

import curses

import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import launcher as ui                 # noqa: E402

MENU = ["Launch", "---", "Update", "Rebuild", "---", "Infos", "Exit", "---",
        "Keyboard layout"]
SEPARATOR = lambda i: MENU[i] == "---"                          # noqa: E731


class FakeScreen:
    """Records what each frame draws, and replays a scripted key sequence."""

    def __init__(self, h, w, keys):
        self.h, self.w, self.keys = h, w, list(keys)
        self.frame, self.frames = {}, []

    def getmaxyx(self):
        return (self.h, self.w)

    def erase(self):
        self.frame = {}

    def refresh(self):
        self.frames.append(self.frame)

    def timeout(self, ms):
        pass

    def addstr(self, y, x, text, attr=0):
        if y >= self.h or x >= self.w:
            raise curses.error
        self.frame.setdefault(y, []).append((x, text))

    def getch(self):
        return self.keys.pop(0) if self.keys else ord("q")


@pytest.fixture
def no_colors(monkeypatch):
    """curses.color_pair() needs initscr(); off-screen drawing does not."""
    monkeypatch.setattr(curses, "color_pair", lambda n: 0)


# -- cursor movement


def test_steps_over_separators():
    assert ui.step_row(0, 1, len(MENU), SEPARATOR) == 2
    assert ui.step_row(2, -1, len(MENU), SEPARATOR) == 0
    assert ui.step_row(6, 1, len(MENU), SEPARATOR) == 8


def test_held_key_stops_at_either_end():
    assert ui.step_row(0, -1, len(MENU), SEPARATOR, repeated=True) == 0
    assert ui.step_row(8, 1, len(MENU), SEPARATOR, repeated=True) == 8


def test_fresh_press_from_an_end_wraps_to_the_other():
    assert ui.step_row(0, -1, len(MENU), SEPARATOR, repeated=False) == 8
    assert ui.step_row(8, 1, len(MENU), SEPARATOR, repeated=False) == 0


def test_a_separator_at_the_end_does_not_spin():
    tail = ["A", "B", "---"]
    skip = lambda i: tail[i] == "---"                           # noqa: E731
    assert ui.step_row(1, 1, 3, skip, repeated=True) == 1
    assert ui.step_row(1, 1, 3, skip, repeated=False) == 0


@pytest.mark.parametrize("count", [0, 1])
def test_degenerate_lists_hold(count):
    assert ui.step_row(0, -1, count) == 0
    assert ui.step_row(0, 1, count) == 0


def test_every_row_skipped_holds():
    assert ui.step_row(0, 1, 3, skip=lambda i: True) == 0


# -- telling a held arrow from a fresh press


def test_first_press_is_never_a_repeat():
    assert ui.NavRepeat().held(curses.KEY_DOWN) is False


def test_back_to_back_presses_read_as_auto_repeat():
    nav = ui.NavRepeat()
    nav.held(curses.KEY_DOWN)
    assert nav.held(curses.KEY_DOWN) is True


def test_a_different_key_resets():
    nav = ui.NavRepeat()
    nav.held(curses.KEY_DOWN)
    assert nav.held(curses.KEY_UP) is False


def test_a_press_after_the_gap_is_deliberate():
    nav = ui.NavRepeat()
    nav.held(curses.KEY_UP)
    time.sleep(ui.NAV_REPEAT_GAP_S + 0.05)
    assert nav.held(curses.KEY_UP) is False


def test_long_press_parks_at_the_last_row_then_wraps():
    nav, idx = ui.NavRepeat(), 5
    for _ in range(12):                    # no pause between them: auto-repeat
        idx = ui.step_row(idx, 1, 6, repeated=nav.held(curses.KEY_DOWN))
    assert idx == 5, "a held arrow ran off the end"
    time.sleep(ui.NAV_REPEAT_GAP_S + 0.05)
    idx = ui.step_row(idx, 1, 6, repeated=nav.held(curses.KEY_DOWN))
    assert idx == 0, "a fresh press at the end did not wrap"


def test_a_buffered_key_is_a_repeat_however_slow_the_frame():
    nav = ui.NavRepeat()
    nav.held(curses.KEY_DOWN)
    time.sleep(ui.NAV_REPEAT_GAP_S + 0.05)   # a frame slower than the gap
    assert nav.held(curses.KEY_DOWN, buffered=True) is True, \
        "a slow frame broke the hold"


def test_a_pause_after_a_burst_still_reads_as_a_fresh_press():
    """The buffered flag must not latch: it describes one key, not the run."""
    nav = ui.NavRepeat()
    nav.held(curses.KEY_DOWN, buffered=True)
    time.sleep(ui.NAV_REPEAT_GAP_S + 0.05)
    assert nav.held(curses.KEY_DOWN, buffered=False) is False, \
        "a deliberate press after a burst still read as held"


# -- draining a queued burst


class BurstScreen(FakeScreen):
    """Only the input half of a screen: a key buffer and pushback."""

    def __init__(self, keys):
        super().__init__(24, 100, keys)
        self.pushed = []

    def getch(self):
        if self.pushed:
            return self.pushed.pop()
        return self.keys.pop(0) if self.keys else -1

    def ungetch(self, k):
        self.pushed.append(k)


@pytest.fixture
def pushback(monkeypatch):
    """curses.ungetch() needs initscr(); route it back to the fake screen."""
    holder = {}
    monkeypatch.setattr(curses, "ungetch", lambda k: holder["screen"].ungetch(k))
    return holder


def test_a_queued_burst_is_spent_in_one_frame(pushback):
    """Twelve queued Downs move twelve rows now, not one row per redraw."""
    screen = BurstScreen([curses.KEY_DOWN] * 11)
    pushback["screen"] = screen
    idx = ui.nav_step(screen, ui.NavRepeat(), curses.KEY_DOWN, True, 0, 1, 40)
    assert idx == 12
    assert screen.getch() == -1, "keys left in the buffer"


def test_a_queued_burst_still_stops_at_the_end(pushback):
    screen = BurstScreen([curses.KEY_DOWN] * 30)
    pushback["screen"] = screen
    idx = ui.nav_step(screen, ui.NavRepeat(), curses.KEY_DOWN, False, 0, 1, 10)
    assert idx == 9, "a burst ran off the end and wrapped"


def test_draining_leaves_other_keys_alone(pushback):
    """Enter behind a burst must survive: it applies the configuration."""
    screen = BurstScreen([curses.KEY_DOWN, curses.KEY_DOWN, ord("\n"),
                          curses.KEY_DOWN])
    pushback["screen"] = screen
    idx = ui.nav_step(screen, ui.NavRepeat(), curses.KEY_DOWN, False, 0, 1, 40)
    assert idx == 3, "the burst stopped short"
    assert screen.getch() == ord("\n"), "Enter was swallowed"
    assert screen.getch() == curses.KEY_DOWN, "the trailing key was swallowed"


def test_a_slow_frame_between_bursts_does_not_wrap(pushback):
    """The control screen's redraw is slower than the repeat gap."""
    nav, idx = ui.NavRepeat(), 0
    for frame in range(6):
        screen = BurstScreen([curses.KEY_DOWN] * 3)
        pushback["screen"] = screen
        # Every read after the first finds its key already waiting: the burst
        # outran the redraw.
        idx = ui.nav_step(screen, nav, curses.KEY_DOWN, frame > 0, idx, 1, 10)
        time.sleep(ui.NAV_REPEAT_GAP_S + 0.05)     # a frame slower than the gap
    assert idx == 9, f"a held arrow wrapped across slow frames, landed on {idx}"


def test_the_end_still_wraps_after_a_held_burst(pushback):
    """Release, then one press: the wrap must still be reachable."""
    nav, idx = ui.NavRepeat(), 0
    screen = BurstScreen([curses.KEY_DOWN] * 30)
    pushback["screen"] = screen
    idx = ui.nav_step(screen, nav, curses.KEY_DOWN, False, idx, 1, 10)
    assert idx == 9, "the burst did not park at the end"
    time.sleep(ui.NAV_REPEAT_GAP_S + 0.05)
    screen = BurstScreen([])
    pushback["screen"] = screen
    idx = ui.nav_step(screen, nav, curses.KEY_DOWN, False, idx, 1, 10)
    assert idx == 0, "a deliberate press after the burst did not wrap"


# -- reading a key, and whether it was already waiting


def test_read_key_reports_a_waiting_key_as_buffered():
    screen = BurstScreen([curses.KEY_DOWN])
    assert ui.read_key(screen, -1) == (curses.KEY_DOWN, True)


def test_read_key_reports_a_key_it_had_to_wait_for():
    class Waiting(BurstScreen):
        """Empty on the non-blocking poll, answers on the blocking read."""
        def __init__(self):
            super().__init__([])
            self.polled = False

        def getch(self):
            if not self.polled:
                self.polled = True
                return -1
            return curses.KEY_UP

    assert ui.read_key(Waiting(), -1) == (curses.KEY_UP, False)


# -- cut marks on a scrolled pane


def infos_marks(frame, view):
    """(above, below) for one recorded frame of the Technologies screen."""
    return (any(t == "..." for _, t in frame.get(1, [])),
            any(t == "..." for _, t in frame.get(2 + view, [])))


def test_infos_marks_only_the_cut_side(no_colors):
    h = ui.MIN_TERM_HEIGHT
    view = h - 4
    lines = sum(1 + len(body) + 1 for _, body in ui.model.INFOS)
    assert lines > view, "INFOS now fits one screen — this test proves nothing"

    screen = FakeScreen(h, 100, [curses.KEY_DOWN] + [curses.KEY_NPAGE] * 20)
    ui.infos_screen(screen)
    assert infos_marks(screen.frames[0], view) == (False, True)
    assert infos_marks(screen.frames[1], view) == (True, True)
    assert infos_marks(screen.frames[-1], view) == (True, False)


def test_infos_text_keeps_off_the_mark_rows(no_colors):
    h = ui.MIN_TERM_HEIGHT
    view = h - 4
    screen = FakeScreen(h, 100, [curses.KEY_NPAGE] * 20)
    ui.infos_screen(screen)
    allowed = {0, 1, 2 + view, h - 1} | set(range(2, 2 + view))
    for i, frame in enumerate(screen.frames):
        assert not set(frame) - allowed, f"frame {i} drew outside the layout"


def test_infos_last_line_is_still_reachable(no_colors):
    screen = FakeScreen(ui.MIN_TERM_HEIGHT, 100, [curses.KEY_NPAGE] * 20)
    ui.infos_screen(screen)
    tail = ui.model.INFOS[-1][1][-1]
    drawn = [t for row in screen.frames[-1].values() for _, t in row]
    assert any(tail in t for t in drawn), "the pane hid the last line of text"
