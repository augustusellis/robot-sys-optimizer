"""v12 #28 in headless Edge: typing four decimals into a mm field (the package builder's X travel)
reaches the server unrounded (viser rounded it to the arrow step's decimals before)."""

import time

import pytest
import viser

from tests.e2e import cdp

pytestmark = [pytest.mark.e2e, pytest.mark.skipif(not cdp.available(), reason="needs Edge + websockets")]

PORT, DEBUG = 8989, 9389


@pytest.fixture(scope="module")
def app():
    from app import build_app

    server = viser.ViserServer(host="127.0.0.1", port=PORT, verbose=False)
    st = build_app(server)
    yield st
    st["builder_window"].stop()
    server.stop()


FOCUS = (
    "(()=>{const l=[...document.querySelectorAll('label,div,span,p')].filter(e=>e.textContent.trim()=="
    "'X travel (mm)').pop();if(!l)return 'no label';let row=l;for(let k=0;k<6&&row;k++){"
    "const ins=row.querySelectorAll('input');if(ins.length>=2){const i=ins[0];i.scrollIntoView("
    "{block:'center'});i.focus();i.select();return true}row=row.parentElement}return 'no input'})()"
)


def test_typed_four_decimals_reach_the_server(app):
    from ui.package_builder import KIND_OF

    w = app["builder_window"]
    bld = w.builder
    bld.kind.value = KIND_OF["rail"]  # travel fields are shown for carriers
    t_end = time.time() + 10
    while bld.pkg.kind != "rail" and time.time() < t_end:
        time.sleep(0.1)
    with cdp.Browser(f"http://127.0.0.1:{w.port}/", port=DEBUG) as b:
        b.wait_for("document.body.innerText.includes('X travel (mm)')", 40)
        time.sleep(1.0)
        assert b.eval(FOCUS) is True
        b.cdp("Input.insertText", text="-1234.5678")
        b.key("Tab")
        t_end = time.time() + 10
        while bld.tx.value[0] != pytest.approx(-1234.5678) and time.time() < t_end:
            time.sleep(0.2)
        assert bld.tx.value[0] == pytest.approx(-1234.5678, abs=1e-9)
