/* WebSocket client base per VR + Dashboard
 * NOTE [RPI-SAFE-REFACTOR-PHASE1]: modulo analizzato, no functional changes.
 * CORE: j5_connect_ws
 */
export function j5_connect_ws(url, on_msg, on_open, on_close) {
    let ws = new WebSocket(url);
    ws.onopen = on_open;
    ws.onclose = on_close;
    ws.onmessage = (ev) => {
        try { on_msg(JSON.parse(ev.data)); }
        catch(e){ console.error("Invalid WS msg", e); }
    };
    return ws;
}
