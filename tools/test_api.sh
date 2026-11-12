#!/usr/bin/env bash
# HTTP smoke tests for agent_control_api. Run while tools/run_sim.sh is active.
set -u

BASE="${BASE_URL:-http://127.0.0.1:8100}"
PASS=0
FAIL=0

check() {
    local name="$1"
    local expected="$2"
    local actual="$3"
    if [[ "$actual" == "$expected" ]]; then
        echo "PASS  $name -> $actual"
        PASS=$((PASS + 1))
    else
        echo "FAIL  $name -> $actual (expected $expected)"
        FAIL=$((FAIL + 1))
    fi
}

code() {
    curl -sS -o /dev/null -w '%{http_code}' --noproxy '*' "$@"
}

check "GET /" 200 "$(code "$BASE/")"
check "GET /api/health" 200 "$(code "$BASE/api/health")"
check "GET /api/status" 200 "$(code "$BASE/api/status")"
check "GET missing" 404 "$(code "$BASE/no-such-path")"
check "GET job missing" 404 "$(code "$BASE/api/jobs/does-not-exist")"
check "GET bad job id" 400 "$(code "$BASE/api/jobs/bad.id")"
check "POST unknown endpoint" 404 "$(code -X POST \
    -H 'Content-Type: application/json' -d '{}' "$BASE/api/nope")"
check "POST wrong content type" 415 "$(code -X POST \
    -H 'Content-Type: text/plain' -d '{}' "$BASE/api/move")"
check "POST invalid json" 400 "$(code -X POST \
    -H 'Content-Type: application/json' -d '{bad json' "$BASE/api/move")"
check "POST bad direction" 400 "$(code -X POST \
    -H 'Content-Type: application/json' \
    -d '{"direction":"up","distance_m":0.2}' "$BASE/api/move")"
check "POST distance 0" 422 "$(code -X POST \
    -H 'Content-Type: application/json' \
    -d '{"direction":"forward","distance_m":0.0}' "$BASE/api/move")"
check "POST distance 6" 422 "$(code -X POST \
    -H 'Content-Type: application/json' \
    -d '{"direction":"forward","distance_m":6.0}' "$BASE/api/move")"
check "POST lateral 4" 422 "$(code -X POST \
    -H 'Content-Type: application/json' \
    -d '{"direction":"left","distance_m":4.0}' "$BASE/api/move")"
check "POST angle 181" 422 "$(code -X POST \
    -H 'Content-Type: application/json' \
    -d '{"direction":"left","angle_deg":181}' "$BASE/api/turn")"
check "POST bool distance" 400 "$(code -X POST \
    -H 'Content-Type: application/json' \
    -d '{"direction":"forward","distance_m":true}' "$BASE/api/move")"
check "POST unknown command" 400 "$(code -X POST \
    -H 'Content-Type: application/json' \
    -d '{"command":"jump"}' "$BASE/api/command")"
check "POST stop idle" 202 "$(code -X POST \
    -H 'Content-Type: application/json' -d '{}' "$BASE/api/stop")"

BIG_BODY="$(python3 -c "print('{\"direction\":\"forward\",\"distance_m\":0.2,\"pad\":\"' + 'x' * 5000 + '\"}')")"
check "POST oversized" 413 "$(code -X POST \
    -H 'Content-Type: application/json' --data-binary "$BIG_BODY" "$BASE/api/move")"

echo
echo "Summary: $PASS passed, $FAIL failed"
[[ "$FAIL" -eq 0 ]]
