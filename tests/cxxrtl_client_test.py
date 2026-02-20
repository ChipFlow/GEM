# /// script
# /// dependencies = [""]
# ///
"""
CXXRTL protocol client test for the Loom server.

Usage:
    uv run tests/cxxrtl_client_test.py [host:port]

Connects to a running `loom serve` instance and exercises the full
CXXRTL protocol: greeting, list_scopes, list_items, reference_items,
get_simulation_status, run_simulation, and query_interval.

Default address: 127.0.0.1:9000
"""

import json
import socket
import sys


class CxxrtlClient:
    """Minimal CXXRTL protocol client."""

    def __init__(self, host: str, port: int):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.connect((host, port))
        self.buf = b""

    def send(self, msg: dict):
        data = json.dumps(msg).encode("utf-8") + b"\x00"
        self.sock.sendall(data)

    def recv(self) -> dict:
        while b"\x00" not in self.buf:
            chunk = self.sock.recv(65536)
            if not chunk:
                raise ConnectionError("Server closed connection")
            self.buf += chunk
        idx = self.buf.index(b"\x00")
        msg_bytes = self.buf[:idx]
        self.buf = self.buf[idx + 1:]
        return json.loads(msg_bytes.decode("utf-8"))

    def close(self):
        self.sock.close()


def test_protocol(host: str, port: int):
    print(f"Connecting to {host}:{port}...")
    client = CxxrtlClient(host, port)

    # 1. Greeting
    print("\n--- Greeting ---")
    client.send({"type": "greeting", "version": 0})
    greeting = client.recv()
    assert greeting["type"] == "greeting", f"Expected greeting, got {greeting['type']}"
    assert greeting["version"] == 0
    print(f"  Server version: {greeting['version']}")
    print(f"  Commands: {greeting['commands']}")
    print(f"  Events: {greeting['events']}")
    print(f"  Features: {greeting['features']}")

    # 2. List scopes (root)
    print("\n--- List Scopes (root) ---")
    client.send({"type": "command", "command": "list_scopes", "scope": None})
    resp = client.recv()
    assert resp["type"] == "response"
    assert resp["command"] == "list_scopes"
    scopes = resp["scopes"]
    print(f"  Found {len(scopes)} child scope(s)")
    for name, info in list(scopes.items())[:10]:
        print(f"    '{name}': type={info['type']}")
    if len(scopes) > 10:
        print(f"    ... and {len(scopes) - 10} more")

    # 3. List items (root scope)
    print("\n--- List Items (root) ---")
    client.send({"type": "command", "command": "list_items", "scope": ""})
    resp = client.recv()
    assert resp["type"] == "response"
    assert resp["command"] == "list_items"
    items = resp["items"]
    print(f"  Found {len(items)} item(s) in root scope")
    for name, info in list(items.items())[:10]:
        width = info.get("width", "?")
        itype = info.get("type", "?")
        is_input = info.get("input", False)
        is_output = info.get("output", False)
        direction = "input" if is_input else ("output" if is_output else "internal")
        print(f"    '{name}': {itype} width={width} {direction}")
    if len(items) > 10:
        print(f"    ... and {len(items) - 10} more")

    # 4. List all items (scope=null)
    print("\n--- List All Items ---")
    client.send({"type": "command", "command": "list_items", "scope": None})
    resp = client.recv()
    all_items = resp["items"]
    print(f"  Found {len(all_items)} total item(s)")

    # 5. Get simulation status
    print("\n--- Simulation Status ---")
    client.send({"type": "command", "command": "get_simulation_status"})
    resp = client.recv()
    assert resp["type"] == "response"
    print(f"  Status: {resp['status']}")
    print(f"  Latest time: {resp['latest_time']}")

    # 6. Reference items (bind some signals)
    if items:
        item_names = list(items.keys())[:5]
        print(f"\n--- Reference Items ---")
        print(f"  Binding {len(item_names)} signal(s) to ref 'test'")
        client.send({
            "type": "command",
            "command": "reference_items",
            "reference": "test",
            "items": [[name] for name in item_names],
        })
        resp = client.recv()
        assert resp["type"] == "response"
        print(f"  OK: reference bound")

        # 7. Query interval
        print("\n--- Query Interval ---")
        client.send({
            "type": "command",
            "command": "query_interval",
            "interval": ["0.0", "0.1000000000000000"],
            "collapse": True,
            "items": "test",
            "item_values_encoding": "base64(u32)",
            "diagnostics": True,
        })
        resp = client.recv()
        assert resp["type"] == "response"
        assert resp["command"] == "query_interval"
        samples = resp["samples"]
        print(f"  Got {len(samples)} sample(s)")
        for s in samples[:5]:
            diags = s.get("diagnostics", [])
            diag_str = f" ({len(diags)} diagnostics)" if diags else ""
            vals = s.get("item_values", "")
            val_preview = vals[:20] + "..." if len(vals) > 20 else vals
            print(f"    time={s['time']} values={val_preview}{diag_str}")
        if len(samples) > 5:
            print(f"    ... and {len(samples) - 5} more")

        # 8. Deallocate reference
        print("\n--- Deallocate Reference ---")
        client.send({
            "type": "command",
            "command": "reference_items",
            "reference": "test",
            "items": None,
        })
        resp = client.recv()
        assert resp["type"] == "response"
        print("  OK: reference deallocated")

    # 9. Run simulation (should immediately finish in replay mode)
    print("\n--- Run Simulation ---")
    client.send({
        "type": "command",
        "command": "run_simulation",
        "until_time": None,
        "until_diagnostics": [],
        "sample_item_values": True,
    })
    resp = client.recv()
    assert resp["type"] == "response"
    assert resp["command"] == "run_simulation"
    print("  run_simulation acknowledged")

    # Should get a simulation_finished event
    event = client.recv()
    assert event["type"] == "event"
    assert event["event"] == "simulation_finished"
    print(f"  Event: {event['event']} at time {event['time']}")

    print("\n=== All protocol tests passed! ===")
    client.close()


if __name__ == "__main__":
    addr = sys.argv[1] if len(sys.argv) > 1 else "127.0.0.1:9000"
    host, port = addr.rsplit(":", 1)
    test_protocol(host, int(port))
