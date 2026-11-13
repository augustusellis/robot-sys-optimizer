"""V10 task 8 in headless Edge: the package builder's HTML tab bar (click -> hidden-input bridge ->
server) and the status strip above the tabs."""

import time

import pytest
import viser

from tests.e2e import cdp

pytestmark = [pytest.mark.e2e, pytest.mark.skipif(not cdp.available(), reason="needs Edge + websockets")]

PORT, DEBUG, DEBUG2 = 8971, 9371, 9372


@pytest.fixture(scope="module")
def app():
    from app import build_app

    server = viser.ViserServer(host="127.0.0.1", port=PORT, verbose=False)
    st = build_app(server)
    assert st["project"].load_example("rail_long_part")
    yield st
    st["builder_window"].stop()
    server.stop()


def _pos(b, label: str):
    return b.eval(
        "(()=>{const h=[...document.querySelectorAll('button')].filter(e=>e.textContent.trim()==="
        f"'{label}').pop();if(!h)return null;h.scrollIntoView({{block:'center'}});"
        "const r=h.getBoundingClientRect();return [r.left+r.width/2,r.top+r.height/2]})()"
    )


def test_builder_tab_bar_in_a_browser(app):
    w = app["builder_window"]
    bld = w.builder
    with cdp.Browser(f"http://127.0.0.1:{w.port}/", port=DEBUG) as b:
        b.wait_for("[...document.querySelectorAll('button')].some(e=>e.textContent.trim()=='Gantry')", 40)
        time.sleep(1.0)
        # the bridge input's row is hidden by the bar
        hidden = b.eval(
            "(()=>{const i=[...document.querySelectorAll('input')].find(i=>i.value.indexOf('tab:')==0);"
            "if(!i)return 'no input';let e=i;while(e&&e!==document.body){if(getComputedStyle(e).display"
            "=='none')return true;e=e.parentElement}return false})()"
        )
        assert hidden is True, hidden
        assert bld.pkg.kind == "tool"
        x, y = _pos(b, "Gantry")
        b.click(x, y)
        t_end = time.time() + 10
        while bld.pkg.kind != "gantry" and time.time() < t_end:
            time.sleep(0.2)
        assert bld.pkg.kind == "gantry"
        # the clicked tab is drawn as the active one
        assert b.wait_for(
            "(()=>{const h=[...document.querySelectorAll('button')].find(e=>e.textContent.trim()=='Gantry');"
            "return h&&getComputedStyle(h).color=='rgb(255, 255, 255)'})()",
            10,
        )


def test_status_strip_renders(app):
    with cdp.Browser(f"http://127.0.0.1:{PORT}/", port=DEBUG2) as b:
        text = b.wait_for(
            "(()=>{const t=document.body.innerText;return t.indexOf('1 Robot & cell')>=0&&"
            "t.indexOf('no plan yet')>=0?t:''})()",
            40,
        )
        assert "○ 1 Robot & cell" in text or "✔ 1 Robot & cell" in text
        assert "<small>" not in text
