using Unity.Entities;
using UnityEngine;
using Unity.Mathematics;
using Unity.Collections;
using Unity.Jobs;
using Unity.Rendering;
using Unity.Transforms;
using Unity.Physics;
using CapsuleCollider = UnityEngine.CapsuleCollider;
using Material = Unity.Physics.Material;

internal class RopeAuthor : MonoBehaviour
{
    public Vector3 Start = Vector3.zero;
    public Vector3 End = Vector3.up;

    class Baker : Baker<RopeAuthor>
    {
        public override void Bake(RopeAuthor auth)
        {
            AddComponent(new RopeInfo
            {
                Start = auth.Start,
                End = auth.End
            });
        }
    }

    void OnDrawGizmos()
    {
        Gizmos.color = Color.red;
        Gizmos.DrawSphere(Start, 0.1f);
        Gizmos.DrawSphere(End, 0.1f);
        Gizmos.DrawLine(Start, End);
    }
}

internal struct RopeInfo : IComponentData
{
    public float3 Start;
    public float3 End;
    public float Length() => math.distance(Start, End);
}

public partial struct RopeSystem : ISystem
{
    public void OnCreate(ref SystemState state){}
    public void OnDestroy(ref SystemState state){}
    public void OnUpdate(ref SystemState state) {
        if (Input.GetKeyDown(KeyCode.R)) {
            Debug.Log("Rope key pressed");
            // move the rope
            foreach (var ropeRef in SystemAPI.Query<RefRW<RopeInfo>>()) {
                ref var rope = ref ropeRef.ValueRW;
                rope.Start += math.up() * 0.5f;
                rope.End -= 0.6f;
            }
        }

        var ecb = SystemAPI.GetSingleton<EndSimulationEntityCommandBufferSystem.Singleton>().CreateCommandBuffer(state.WorldUnmanaged);

        foreach (var ropeRef in SystemAPI.Query<RefRO<RopeInfo>>().WithChangeFilter<RopeInfo>())
        {
            var rope = ropeRef.ValueRO;
            // generate vertices
            var job = new GenerateRopeVerticesJob(
                math.distance(rope.Start, rope.End), state.WorldUpdateAllocator
            );
            job.Run(job.RingCount);

            // generate indices
            var indices = new NativeArray<int>(job.IndexCount + job.IndexCountCaps, Allocator.TempJob);
            for (var i = 0; i < job.RingCount-1; i++)
            {
                var ringStart = i * job.VerticesPerRing;
                var nextRingStart = (i + 1) * job.VerticesPerRing;
                for (var j = 0; j < job.VerticesPerRing; j++)
                {
                    var nextJ = (j + 1) % job.VerticesPerRing;
                    var index = i * job.VerticesPerRing * 6 + j * 6;
                    indices[index] = ringStart + j;
                    indices[index + 1] = ringStart + nextJ;
                    indices[index + 2] = nextRingStart + j;
                    indices[index + 3] = nextRingStart + j;
                    indices[index + 4] = ringStart + nextJ;
                    indices[index + 5] = nextRingStart + nextJ;
                }
            }
            // start cap
            for (var i = 0; i < job.VerticesPerRing-1; i++)
            {
                var nextI = (i + 1) % job.VerticesPerRing;
                var index = job.IndexCount + i * 3;
                indices[index] = 0;
                indices[index + 1] = nextI;
                indices[index + 2] = i;
            }
            // end cap
            for (var i = 0; i < job.VerticesPerRing-1; i++)
            {
                var nextI = (i + 1) % job.VerticesPerRing;
                var index = job.IndexCount + job.IndexCountCaps/2 + i * 3;
                indices[index] = job.Vertices.Length - 1;
                indices[index + 1] = job.Vertices.Length - 1 - nextI;
                indices[index + 2] = job.Vertices.Length - 1 - i;
            }

            // create bone weights
            var boneWeights = new NativeArray<BoneWeight>(job.Vertices.Length, Allocator.TempJob);
            for (var i = 0; i < job.Vertices.Length; i++)
            {
                var ringIndex = i/job.VerticesPerRing;
                var segmentIndex = (ringIndex-1)/2;
                var boneIndex0 = segmentIndex;

                boneWeights[i] = new BoneWeight
                {
                    weight0 = 1f,
                    weight1 = 0f,
                    weight2 = 0f,
                    weight3 = 0f,
                    boneIndex0 = math.clamp(boneIndex0, 0, job.SegmentCount-2),
                    boneIndex1 = 0,
                    boneIndex2 = 0,
                    boneIndex3 = 0
                };
            }

            // create bones
            var bones = new Transform[job.SegmentCount-1];
            for (var i = 0; i < bones.Length; i++)
            {
                var bone = new GameObject("Rope bone " + i);
                bone.transform.position = math.lerp(rope.Start, rope.End, (float)i/bones.Length);
                bone.transform.rotation = Quaternion.LookRotation(rope.End - rope.Start);
                bones[i] = bone.transform;
                //Debug.DrawLine(bone.transform.position, bone.transform.position + Vector3.up, Color.red, 10f);
            }

            // create bind poses
            var bindPoses = new NativeArray<Matrix4x4>(bones.Length, Allocator.TempJob);
            for (var i = 0; i < bones.Length; i++) {
                var point = new float3(0, 0, (math.distance(rope.Start, rope.End) / bones.Length) * i);
                // Debug.DrawLine(point, point + math.up()*0.1f, Color.magenta, 10f);
                bindPoses[i] = LocalTransform.FromPosition(point).Inverse().ToMatrix();
            }

            // create physic geometry for bones
            var colliderLength = rope.Length() / bones.Length;

            // physic geometry for bones as CapsuleCollider
            var capsuleCollider = Unity.Physics.CapsuleCollider.Create(new CapsuleGeometry {
                Radius = 0.1f,
                Vertex0 = 0,
                Vertex1 = new float3(0f, 0f, colliderLength-0.1f),
            }, CollisionFilter.Default, new Material {
                CollisionResponse = CollisionResponsePolicy.Collide,
                FrictionCombinePolicy = Material.CombinePolicy.Minimum,
                Friction = 0f,
                RestitutionCombinePolicy = Material.CombinePolicy.Minimum,
                Restitution = 0.0f
            });

            // create bone entities with physics
            var boneEntities = new NativeArray<Entity>(bones.Length, Allocator.TempJob);
            var boneArchetypeComponentList = new FixedList128Bytes<ComponentType> {
                ComponentType.ReadOnly<LocalToWorld>(), 
                ComponentType.ReadOnly<LocalTransform>(),
                // working collider
                ComponentType.ReadOnly<PhysicsCollider>(),
                ComponentType.ReadOnly<PhysicsWorldIndex>(),
                // because dynamic
                ComponentType.ReadOnly<PhysicsVelocity>(),
                ComponentType.ReadOnly<PhysicsMass>(),
                ComponentType.ReadOnly<PhysicsDamping>(),
                ComponentType.ReadOnly<PhysicsGravityFactor>(),
            }.ToNativeArray(state.WorldUpdateAllocator);
            var boneArchetype = state.EntityManager.CreateArchetype(boneArchetypeComponentList);
            state.EntityManager.CreateEntity(boneArchetype, boneEntities);
            
            for (var i = 0; i < boneEntities.Length; i++){
                // Set the transform of the entity to the transform of the bone
                state.EntityManager.SetName(boneEntities[i], bones[i].name);
                state.EntityManager.SetComponentData(boneEntities[i], LocalTransform.FromMatrix(bones[i].localToWorldMatrix));
                ecb.AddComponent(boneEntities[i], new CompanionLink{Companion = bones[i].gameObject});
                
                // set collider
                state.EntityManager.SetComponentData(boneEntities[i], new PhysicsCollider{Value = capsuleCollider});

                // set physics
                state.EntityManager.SetComponentData(boneEntities[i], PhysicsMass.CreateDynamic(capsuleCollider.Value.MassProperties, 0.001f));
                state.EntityManager.SetComponentData(boneEntities[i], new PhysicsDamping{Linear = 0.1f, Angular = 0.1f});
                state.EntityManager.SetComponentData(boneEntities[i], new PhysicsGravityFactor{Value = 1f});
            }
            
            // create rope joints
            var ropeJointEntities = new NativeArray<Entity>((bones.Length-1)*2, Allocator.TempJob);
            var ropeJointArchetypeComponentList = new FixedList128Bytes<ComponentType> {
                ComponentType.ReadOnly<PhysicsConstrainedBodyPair>(),
                ComponentType.ReadOnly<PhysicsWorldIndex>(),
                ComponentType.ReadOnly<PhysicsJoint>(),
                ComponentType.ReadOnly<PhysicsJointCompanion>(),
            }.ToNativeArray(state.WorldUpdateAllocator);
            var ropeJointArchetype = state.EntityManager.CreateArchetype(ropeJointArchetypeComponentList);
            state.EntityManager.CreateEntity(ropeJointArchetype, ropeJointEntities);
            for (var i = 0; i < bones.Length-1; i++){
                // make sure rope doesn't twist
                var bodyA = new BodyFrame {
                    Position = new float3(0, 0, colliderLength),
                    Axis = new float3(1,0,0),
                    PerpendicularAxis = new float3(0,1,0),
                };
                PhysicsJoint.CreateRagdoll(bodyA, BodyFrame.Identity,
                    math.PI / 2f,
                    new Math.FloatRange(-math.PI / 16f, math.PI / 16f),
                    new Math.FloatRange(-math.PI / 16f, math.PI / 16f), 
                    out var primaryConeAndTwist, out var perpendicularCone);
                
                // create two joints for each bone
                state.EntityManager.SetName(ropeJointEntities[i*2], "Rope joint " + i + " A");
                state.EntityManager.SetComponentData(ropeJointEntities[i*2], new PhysicsConstrainedBodyPair(boneEntities[i], boneEntities[i+1], false));
                state.EntityManager.GetBuffer<PhysicsJointCompanion>(ropeJointEntities[i*2]).Add(new PhysicsJointCompanion{JointEntity = ropeJointEntities[i*2+1]});
                state.EntityManager.SetComponentData(ropeJointEntities[i*2], primaryConeAndTwist);
                
                state.EntityManager.SetName(ropeJointEntities[i*2+1], "Rope joint " + i + " B");
                state.EntityManager.SetComponentData(ropeJointEntities[i*2+1], new PhysicsConstrainedBodyPair(boneEntities[i], boneEntities[i+1], false));
                state.EntityManager.SetComponentData(ropeJointEntities[i*2+1], perpendicularCone);
            }

            // constrain to point on start
            var startJointEntity = state.EntityManager.CreateEntity(ropeJointArchetype);
            state.EntityManager.SetName(startJointEntity, "Rope start joint");
            state.EntityManager.SetComponentData(startJointEntity, new PhysicsConstrainedBodyPair(boneEntities[0], Entity.Null, true));
            state.EntityManager.SetComponentData(startJointEntity, PhysicsJoint.CreateBallAndSocket(0, rope.Start));

            // constrain to point on end
            var endJointEntity = state.EntityManager.CreateEntity(ropeJointArchetype);
            state.EntityManager.SetName(endJointEntity, "Rope end joint");
            state.EntityManager.SetComponentData(endJointEntity, new PhysicsConstrainedBodyPair(boneEntities[^1], Entity.Null, true));
            state.EntityManager.SetComponentData(endJointEntity, PhysicsJoint.CreateBallAndSocket(new float3(0, 0, colliderLength), rope.End+math.down()*2));
            
            // generate mesh
            var mesh = new Mesh();
            mesh.MarkDynamic();
            mesh.name = "Rope Mesh";
            mesh.SetVertices(job.Vertices);
            mesh.SetIndices(indices, MeshTopology.Triangles, 0);
            mesh.RecalculateNormals();
            mesh.bounds = new Bounds(Vector3.zero, Vector3.one * 1000f);
            mesh.boneWeights = boneWeights.ToArray();
            mesh.bindposes = bindPoses.ToArray();

            // Draw the mesh
            var go = new GameObject("Rope");
            var mr = go.AddComponent<SkinnedMeshRenderer>();
            // get urp material
            var urp = UnityEngine.Rendering.Universal.UniversalRenderPipeline.asset;
            mr.material = urp.defaultMaterial;
            mr.sharedMesh = mesh;
            mr.material.color = new Color(0.5f, 0.3f, 0.1f);
            mr.bones = bones;

            // draw all verts
            // foreach (var vert in job.Vertices) {
            //     Debug.DrawLine(vert, vert + math.up() * 0.14f, Color.red, 20f);
            // }
            
            indices.Dispose();
            job.Vertices.Dispose();
            boneWeights.Dispose();
            bindPoses.Dispose();
            Debug.Log("Rope vertices generated");
        }
    }
}

// Create cylinder verts job
//[Unity.Burst.BurstCompile]
internal struct GenerateRopeVerticesJob : IJobFor {
    public readonly int SegmentCount => math.max((int)(length/metersPerSegment), 2);
    readonly float metersPerSegment; // number of segments
    public readonly int RingCount => SegmentCount * 2;
    public int VerticesPerRing { get; }

    readonly float middleRadius;
    readonly float capRadius;
    readonly float length; // in meters

    readonly float deltaOut;

    [NativeDisableParallelForRestriction] [WriteOnly]
    NativeArray<float3> vertices; // we know the size is RingCount * CircleVertexCount


    public readonly NativeArray<float3> Vertices => vertices;
    public readonly int VertexCount => vertices.Length;

    public readonly int IndexCount => RingCount * VerticesPerRing * 6;
    public readonly int IndexCountCaps => (VerticesPerRing-1) * 2 * 3;

    public GenerateRopeVerticesJob(float length, Allocator allocator = Allocator.TempJob, float metersPerSegment = 0.8f, int verticesPerRing = 9, float middleRadius = 0.1f, float capRadius = 0.05f, float deltaOut = 0.1f) {
        this.metersPerSegment = metersPerSegment;
        VerticesPerRing = verticesPerRing;
        this.middleRadius = middleRadius;
        this.capRadius = capRadius;
        this.deltaOut = deltaOut;
        var ringCount = math.max((int)(length/metersPerSegment), 2)*2; // 2 rings per segment
        vertices = CollectionHelper.CreateNativeArray<float3>(ringCount * verticesPerRing, allocator);
        this.length = length;
    }

    // 0 is start ring, SegmentCount*2 is end ring
    public void Execute(int ringIndex) {
        // create end and start ring
        if (ringIndex == 0 || ringIndex == RingCount-1) {
            // create vertices
            var radius = capRadius;
            var startIndex = ringIndex * VerticesPerRing;
            var isFirst = ringIndex % 2 == 0;
            var delta = isFirst ? -deltaOut : deltaOut;
            var startPos = new float3(0, 0, (ringIndex == 0 ? 0 : length) + delta);

            // create ring
            for (var i = 0; i < VerticesPerRing; i++) {
                var angle = (float)i / VerticesPerRing * math.PI * 2;
                var x = math.cos(angle) * radius;
                var y = math.sin(angle) * radius;
                var pos = new float3(x, y, 0);
                vertices[startIndex + i] = pos + startPos;
            }
        } else {
            // create vertices
            var radius = middleRadius;
            var startIndex = ringIndex * VerticesPerRing;
            var isFirst = ringIndex % 2 == 0;
            var delta = isFirst ? -deltaOut : deltaOut;;
            var offset = length*((float)(ringIndex/2) / (SegmentCount-1));
            var startPos = new float3(0, 0, offset + delta);

            // create ring
            for (var i = 0; i < VerticesPerRing; i++) {
                var angle = (float)i / VerticesPerRing * math.PI * 2;
                var x = math.cos(angle) * radius;
                var y = math.sin(angle) * radius;
                var pos = new float3(x, y, 0);
                vertices[startIndex + i] = pos + startPos;
            }
        }
    }
}