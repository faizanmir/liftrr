# LIFTRR BLE Commands and Events

This document lists the BLE commands and events implemented in the current firmware.

Note: All commands may include `phoneEpochMs` in the request body to sync device time.

## Commands

### ping
- Request body (`body`): `{}`
- Response body:
  - `uptimeMs` (uint32)
  - `epochMs` (int64)
  - `fw` (string)

### capabilities.get
- Request body (`body`): `{}`
- Response body:
  - `device.model` (string)
  - `device.fw` (string)
  - `maxMtu` (uint32)
  - `features.time.sync` (bool)
  - `features.mode.set` (bool)
  - `features.session.start` (bool)
  - `features.session.end` (bool)
  - `features.sessions.list` (bool)
  - `features.session.stream` (bool)
  - `features.session.stream.bt_classic` (bool)
  - `features.sessions.clear` (bool)

### time.sync
- Request body (`body`): `{ "phoneEpochMs": <int64> }`
- Response body:
  - `epochAtSyncMs` (int64)
  - `millisAtSyncMs` (uint32)

### mode.set
- Request body (`body`): `{ "mode": "RUN" | "IDLE" | "DUMP" }`
- Response body:
  - `mode` (string)

### session.start
- Request body (`body`): `{ "lift": "<string>", "phoneEpochMs": "<optional int64>" }`
- Response body:
  - `sessionId` (string)
  - `lift` (string)
  - `mode` (string)
- Error: `CALIBRATION_REQUIRED` with body `pending: true` when calibration is needed.
- Notes: if `phoneEpochMs` is provided, the device time is synced before creating the session ID.

### session.end
- Request body (`body`): `{}`
- Response body: none
- Error: `NOT_ACTIVE` if no active session.

### sessions.list
- Request body (`body`): `{ "cursor": <int64>, "limit": <int64> }`
- Response body (BLE): none
- Response via BT classic: JSON line with body fields:
  - `items[]` (array)
  - `nextCursor` (uint32)
  - `hasMore` (bool)
- Notes: BLE response `code` is `SENT_VIA_BT_CLASSIC`; Classic must be connected.

### sessions.clear
- Request body (`body`): `{}`
- Response body: none
- Error: `SESSION_ACTIVE` if a session is active.

### session.stream
- Request body (`body`): `{ "sessionId": "<string>" }`
- Response body:
  - `sessionId` (string)
  - `size` (uint32)
- Notes: requires BT classic connection; file bytes are streamed raw on Classic with no metadata framing.

## Events

### time.sync.request
- Body:
  - `timeoutMs` (uint32)

### time.sync.timeout
- Body:
  - `timeoutMs` (uint32)

### bt_classic.required
- Body:
  - `status` (string, e.g. `not_connected`)

### session.started
- Body:
  - `sessionId` (string)
  - `lift` (string)
  - `auto` (bool)

### orientation.status
- Body:
  - `facing` (string: `UP|DOWN|LEFT|RIGHT`)
  - `ok` (bool)

### calibration.succeeded
- Body:
  - `imu` (bool)
  - `laser` (bool)
  - `ready` (bool)
