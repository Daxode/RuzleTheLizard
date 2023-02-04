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
            var followPoints = AddBuffer<FollowPoint>();
            if (auth.Path == null) return;
            followPoints.EnsureCapacity(auth.Path.Length);
            foreach (var point in auth.Path) {
                followPoints.Add(point);
            }

            if (followPoints.Length < 2) return;
            // get the first two points
            float3 pointA = followPoints[0];
            float3 pointB = followPoints[1];
            var physicsJoint = JointHelpers.CalculateJointBetween(pointA, pointB);
            var jointEntity = JointHelpers.CreateJoint(this, GetEntity(), physicsJoint);

            // Debug.Log("Created joint");
            // // for (int i = 0; i < followPoints.Length - 1; i++) {
            // //     Debug.DrawLine((float3)followPoints[i], (float3)followPoints[i + 1], Color.green, 20f);
            // // }
            AddComponent(new CurrentJoint { Value = jointEntity });
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


// Create system with singleton of whether Q is pressed
partial struct InputSystem : ISystem {
    public struct InputState : IComponentData {
        public FixedInputEvent Q;
    }

    public void OnCreate(ref SystemState state) {
        state.EntityManager.AddComponentData(state.SystemHandle, new InputState());
        state.RequireForUpdate<FixedTickSystem.Singleton>();
    }

    public void OnDestroy(ref SystemState state) {}

    public void OnUpdate(ref SystemState state) {
        var fixedTick = SystemAPI.GetSingleton<FixedTickSystem.Singleton>().Tick;
        var inputState = state.EntityManager.GetComponentDataRW<InputState>(state.SystemHandle);
        if (Input.GetKeyDown(KeyCode.Q))
            inputState.ValueRW.Q.Set(fixedTick);
    }
}


// Update in physics group
[BurstCompile]
[UpdateInGroup(typeof(BeforePhysicsSystemGroup))]
public partial struct PathConstraintSystem : ISystem {
    public void OnCreate(ref SystemState state){
        state.RequireForUpdate<FixedTickSystem.Singleton>();
    }
    public void OnDestroy(ref SystemState state){}

    int i;
    //[BurstCompile]
    public void OnUpdate(ref SystemState state) {
        // Get the input state singleton
        var inputState = SystemAPI.GetSingleton<InputSystem.InputState>();
        if (inputState.Q.IsSet(SystemAPI.GetSingleton<FixedTickSystem.Singleton>().Tick)) {
            foreach (var (followPoints, localTransform, vel, currentJoint) in SystemAPI.Query<DynamicBuffer<FollowPoint>, RefRO<LocalTransform>, RefRO<PhysicsVelocity>, RefRO<CurrentJoint>>()) {
                // get the first two points
                float3 pointA = followPoints[i];
                float3 pointB = followPoints[i+1];
                var physicsJoint = JointHelpers.CalculateJointBetween(pointA, pointB);
                SystemAPI.SetComponent(currentJoint.ValueRO.Value, physicsJoint);
                i++;
                if (i >= followPoints.Length - 1) i = 0;
            }
        }
    }
}

public static class JointHelpers {
    public static Entity CreateJoint(EntityCommandBuffer ecb, Entity entity, PhysicsJoint physicsJoint){
        var constraintBodyPair = new PhysicsConstrainedBodyPair(entity, Entity.Null, false);
        var jointEntity = ecb.CreateEntity();
        ecb.AddSharedComponent(jointEntity, new PhysicsWorldIndex(0));
        ecb.AddComponent(jointEntity, constraintBodyPair);
        ecb.AddComponent(jointEntity, physicsJoint);
        return jointEntity; 
    }

    public static Entity CreateJoint<T>(Baker<T> baker, Entity entity, PhysicsJoint physicsJoint) where T : Component {
        var constraintBodyPair = new PhysicsConstrainedBodyPair(entity, Entity.Null, false);
        var jointEntity = baker.CreateAdditionalEntity();
        baker.AddSharedComponent(jointEntity, new PhysicsWorldIndex(0));
        baker.AddComponent(jointEntity, constraintBodyPair);
        baker.AddComponent(jointEntity, physicsJoint);
        return jointEntity; 
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