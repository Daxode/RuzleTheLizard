using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Entities;
using Unity.Mathematics;
using Unity.Collections;
using Unity.Physics;
using Unity.Physics.Systems;
using Unity.Transforms;
using Rival;
using Unity.Burst;

public class MoveAlongPathAuthor : MonoBehaviour
{
    public Vector3[] Path;
    public float Speed = 1f;

    class Baker : Baker<MoveAlongPathAuthor> {
        public override void Bake(MoveAlongPathAuthor auth) {
            AddComponent(new Speed { Value = auth.Speed });
            var followPoints = AddBuffer<FollowPoint>();
            if (auth.Path == null) return;
            followPoints.EnsureCapacity(auth.Path.Length);
            foreach (var point in auth.Path) {
                followPoints.Add(point);
            }
        }
    }

    void OnDrawGizmos() {
        if (Path == null || Path.Length == 0) return;
        Gizmos.color = Color.red;
        for (int i = 0; i < Path.Length; i++) {
            Gizmos.DrawSphere(Path[i], 0.1f);
            if (i > 0) {
                Gizmos.DrawLine(Path[i - 1], Path[i]);
            }
        }
    }
}


[InternalBufferCapacity(0)]
struct FollowPoint : IBufferElementData {
    public float3 position;

    public static implicit operator FollowPoint(float3 position) {
        return new FollowPoint { position = position };
    }

    // implicit conversion to vector3
    public static implicit operator FollowPoint(Vector3 position) {
        return new FollowPoint { position = position };
    }

    // implicit conversion to float3
    public static implicit operator float3(FollowPoint element) {
        return element.position;
    }
}

struct Speed : IComponentData {
    public float Value;
}

// Update in physics group
[UpdateInGroup(typeof(FixedStepSimulationSystemGroup))]
[BurstCompile]
public partial struct MoveAlongPathSystem : ISystem {
    public void OnCreate(ref SystemState state){}
    public void OnDestroy(ref SystemState state){}

    [BurstCompile]
    public void OnUpdate(ref SystemState state){
        foreach (var (positions, vel, transform, speed, physicsMass) in SystemAPI.Query<DynamicBuffer<FollowPoint>, RefRW<PhysicsVelocity>, RefRO<LocalTransform>, RefRO<Speed>, RefRO<PhysicsMass>>()) {
            var interp = (float)math.sin(SystemAPI.Time.ElapsedTime * math.PI * speed.ValueRO.Value) * 0.5f + 0.5f;

            // get the current position on polyline
            var totalDistancce = 0f;
            for (int i = 0; i < positions.Length - 1; i++) {
                totalDistancce += math.distance(positions[i], positions[i + 1]);
            }

            var currentDistance = totalDistancce * interp;
            var currentPoint = 0;
            var currentPointDistance = 0f;
            for (int i = 0; i < positions.Length - 1; i++) {
                var distance = math.distance(positions[i], positions[i + 1]);
                if (currentDistance > currentPointDistance + distance) {
                    currentPointDistance += distance;
                    currentPoint++;
                } else {
                    break;
                }
            }

            // get the current position on the line
            var lineDistance = math.distance(positions[currentPoint], positions[currentPoint + 1]);
            var lineInterp = (currentDistance - currentPointDistance) / lineDistance;
            var position = math.lerp(positions[currentPoint], positions[currentPoint + 1], lineInterp);

            // set the position
            vel.ValueRW.Linear = position - transform.ValueRO.Position;
            var rigidTransform = new RigidTransform(transform.ValueRO.Rotation, position);
            vel.ValueRW = PhysicsVelocity.CalculateVelocityToTarget(in physicsMass.ValueRO, in transform.ValueRO.Position, in transform.ValueRO.Rotation, in rigidTransform, 1f/SystemAPI.Time.DeltaTime);

            // debug draw the current position
            //Debug.DrawLine(transform.ValueRO.Position, position, Color.red, SystemAPI.Time.DeltaTime);
        }
    }
}