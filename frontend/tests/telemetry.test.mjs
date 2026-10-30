import test from 'node:test';
import assert from 'node:assert/strict';
import { demoFrame, parseFrame, distance } from '../src/telemetry.ts';

const valid = () => demoFrame(23);
const parse = frame => parseFrame(JSON.stringify(frame));

test('accepts shared-world telemetry and computes 3D distances', () => {
  const frame = parse(valid());
  assert.equal(frame.drones.length, 2);
  assert.equal(frame.frame, 'world_ned');
  assert.equal(distance({ x: 0, y: 0, z: 0 }, { x: 3, y: 4, z: 12 }), 13);
});

test('rejects local coordinates, duplicate ids and malformed vectors', () => {
  const local = valid(); local.frame = 'vehicle_local';
  assert.throws(() => parse(local), /world_ned/);
  const duplicate = valid(); duplicate.drones[1].id = duplicate.drones[0].id;
  assert.throws(() => parse(duplicate), /重复/);
  const malformed = valid(); malformed.drones[0].position.x = '5';
  assert.throws(() => parse(malformed), /位置/);
});

test('rejects stale, future, nonfinite and unknown status telemetry', () => {
  const stale = valid(); stale.timestamp = Date.now() - 20_000;
  assert.throws(() => parse(stale), /时间戳/);
  const future = valid(); future.timestamp = Date.now() + 20_000;
  assert.throws(() => parse(future), /时间戳/);
  const numeric = valid(); numeric.drones[0].yaw = Infinity;
  assert.throws(() => parse(numeric), /朝向/);
  const status = valid(); status.drones[0].status = 'flying';
  assert.throws(() => parse(status), /状态/);
});

test('validates map obstacles and rejects executable camera URLs', () => {
  const obstacle = valid(); obstacle.obstacles[0].width = -1;
  assert.throws(() => parse(obstacle), /障碍物/);
  const camera = valid(); camera.drones[0].camera_url = 'javascript:alert(1)';
  assert.throws(() => parse(camera), /HTTP/);
});

test('bounds trail payload and permits missing optional trail and camera', () => {
  const long = valid(); long.drones[0].trail = Array.from({ length: 800 }, (_, i) => ({ x: i, y: 0, z: -1 }));
  assert.equal(parse(long).drones[0].trail.length, 600);
  delete long.drones[0].trail;
  assert.deepEqual(parse(long).drones[0].trail, []);
});

test('demo routes avoid obstacle footprints and trail ends at each drone', () => {
  for (let t = 0; t < 100; t += .25) {
    const frame = demoFrame(t);
    for (const drone of frame.drones) {
      const p = drone.position;
      for (const o of frame.obstacles) {
        assert.ok(!(Math.abs(p.y - o.y) < o.width / 2 && Math.abs(p.x - o.x) < o.depth / 2), `${drone.id} intersects ${o.id} at ${t}s`);
      }
      assert.ok(distance(drone.trail.at(-1), p) < .00001);
    }
  }
});
