using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Entities;
using Unity.Transforms;
using Rival;
using Unity.Mathematics;

public class FollowPlayerLooslyAuthor : MonoBehaviour
{
    public GameObject player;

    class Baker : Baker<FollowPlayerLooslyAuthor>
    {
        public override void Bake(FollowPlayerLooslyAuthor authoring)
        {
            AddComponent(new FollowPlayerLoosly
            {
                player = GetEntity(authoring.player),
            });
        }
    }
}

struct FollowPlayerLoosly : IComponentData
{
    public Entity player;
}


[UpdateInGroup(typeof(SimulationSystemGroup))]
[UpdateAfter(typeof(TransformSystemGroup))]
[UpdateBefore(typeof(OrbitCameraSystem))]
partial struct FollowPlayerLooslySystem : ISystem {
    public void OnCreate(ref SystemState state) {}
    public void OnDestroy(ref SystemState state) {}

    bool shouldCauseTransition;
    public void OnUpdate(ref SystemState state) {
        foreach (var (transform, followplayer) in SystemAPI.Query<RefRW<LocalTransform>, FollowPlayerLoosly>()) {
            var playerTransform = SystemAPI.GetComponent<LocalToWorld>(followplayer.player);
            var desiredPos = playerTransform.Position+new float3(0, 1.5f, 0);

            // if player grounded, follow player
            if (SystemAPI.GetComponent<KinematicCharacterBody>(followplayer.player).IsGrounded) {
                if (shouldCauseTransition) {
                    if (math.abs(transform.ValueRO.Position.y - playerTransform.Position.y) < 0.1f) {
                        shouldCauseTransition = false;
                    }
                    transform.ValueRW.Position = math.lerp(transform.ValueRO.Position, playerTransform.Position+new float3(0, 1.5f, 0), 0.1f);
                }
            } else {
                shouldCauseTransition = true;
                var pos = transform.ValueRO.Position;
                desiredPos.y = pos.y;

                // unless y diff is too big then slowly push camera up
                if (math.abs(pos.y - playerTransform.Position.y) > 0.5f) {
                    desiredPos.y = math.lerp(pos.y, playerTransform.Position.y + 1.5f, 0.1f);
                }
                transform.ValueRW.Position = math.lerp(transform.ValueRO.Position, desiredPos, 0.2f);
            }

            if (!shouldCauseTransition)
                transform.ValueRW.Position = math.lerp(transform.ValueRO.Position, desiredPos, 0.5f);
        }
    }
}