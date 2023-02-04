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

    [BurstCompile]
    public void OnUpdate(ref SystemState state) {

    }
}