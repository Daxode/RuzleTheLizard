using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Entities;
using Unity.Mathematics;
using Unity.Collections;
using Unity.Physics;
using Unity.Physics.Extensions;
using Unity.Physics.SimulationEvents;
using Unity.Physics.Systems;
using Unity.Transforms;
using Rival;
using Unity.Burst;

public class ConstrainToPathAuthor : MonoBehaviour
{
    public Vector3[] Path;

    class Baker : Baker<ConstrainToPathAuthor> {
        public override void Bake(ConstrainToPathAuthor auth) {
            AddComponent<CurrentJoint>();
            AddComponent<NotInitialized>();
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

struct NotInitialized : IComponentData {}
struct CurrentJoint : IComponentData {
    public Entity Value;
}


// Set filter
[WorldSystemFilter(WorldSystemFilterFlags.BakingSystem)]
partial struct PathConstraintBakingSystem : ISystem {
    public void OnCreate(ref SystemState state) {}
    public void OnDestroy(ref SystemState state) {}
    public void OnUpdate(ref SystemState state) {
        // foreach (var (entity, followPoints) in SystemAPI.Query<DynamicBuffer<FollowPoint>>().WithAll<NotInitialized>().WithEntityAccess()) {
        //     PhysicsJoint.CreatePrismatic()
        // }
    }
}


// Update in physics group
[UpdateInGroup(typeof(FixedStepSimulationSystemGroup))]
[BurstCompile]
public partial struct PathConstraintSystem : ISystem {
    public void OnCreate(ref SystemState state){}
    public void OnDestroy(ref SystemState state){}

    //[BurstCompile]
    public void OnUpdate(ref SystemState state) {
        var ecb = SystemAPI.GetSingleton<EndFixedStepSimulationEntityCommandBufferSystem.Singleton>().CreateCommandBuffer(state.WorldUnmanaged);
        // only run if space presssed
        if (!Input.GetKeyDown(KeyCode.Space)) return;

        foreach (var (followPoints, localTransform, entity) in SystemAPI.Query<DynamicBuffer<FollowPoint>, RefRO<LocalTransform>>().WithAll<NotInitialized>().WithEntityAccess()) {
            // continue if follow points is less than 2
            if (followPoints.Length < 2) continue;
            // get the first two points
            float3 pointA = followPoints[0];
            float3 pointB = followPoints[1];
            var physicsJoint = CalculateJointBetween(pointA, pointB);
            var jointEntity = CreateJoint(ecb, entity, physicsJoint);

            Debug.Log("Created joint");
            // for (int i = 0; i < followPoints.Length - 1; i++) {
            //     Debug.DrawLine((float3)followPoints[i], (float3)followPoints[i + 1], Color.green, 20f);
            // }
            ecb.SetComponent(entity, new CurrentJoint { Value = jointEntity });

            ecb.RemoveComponent<NotInitialized>(entity);
        }

        foreach (var (followPoints, localTransform, currentJoint, entity) in SystemAPI.Query<DynamicBuffer<FollowPoint>, RefRO<LocalTransform>, RefRO<CurrentJoint>>().WithEntityAccess().WithNone<NotInitialized>()) {
            // continue if follow points is less than 3
            if (followPoints.Length < 3) continue;
            Debug.Log("Created joint");
            // get the first two points
            float3 pointA = followPoints[1];
            float3 pointB = followPoints[2];
            var physicsJoint = CalculateJointBetween(pointA, pointB);
            SetJoint(currentJoint.ValueRO, physicsJoint, ref state);
        }
    }

    public static Entity CreateJoint(EntityCommandBuffer ecb, Entity entity, PhysicsJoint physicsJoint){
            var constraintBodyPair = new PhysicsConstrainedBodyPair(entity, Entity.Null, false);
            var jointEntity = ecb.CreateEntity();
            ecb.AddSharedComponent(jointEntity, new PhysicsWorldIndex(0));
            ecb.AddComponent(jointEntity, constraintBodyPair);
            ecb.AddComponent(jointEntity, physicsJoint);
            return jointEntity;
    }

    void SetJoint(CurrentJoint currentJoint, PhysicsJoint physicsJoint, ref SystemState state){
        var jointEntity = currentJoint.Value;
        SystemAPI.SetComponent(jointEntity, physicsJoint);
    }

    public static PhysicsJoint CalculateJointBetween(float3 pointA, float3 pointB) {
            // get the direction from pointA to pointB
            var direction = pointB - pointA;
            var distance = math.length(direction);
            var body = new BodyFrame 
            {
                Axis = math.normalize(direction),
                PerpendicularAxis = 1,
            };
            var bodyB = body;
            bodyB.Position = pointA;
            var physicsJoint = PhysicsJoint.CreatePrismatic(
                body, bodyB,
                new float2(0, distance)
            );

            physicsJoint.SetImpulseEventThresholdAllConstraints(math.INFINITY);

            return physicsJoint;
    }
}