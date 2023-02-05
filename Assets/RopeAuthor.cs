using Unity.Entities;
using UnityEngine;
using Unity.Mathematics;
using Unity.Collections;
using Unity.Jobs;
using Unity.Rendering;
using Unity.Transforms;

class RopeAuthor : MonoBehaviour
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
}

struct RopeInfo : IComponentData
{
    public float3 Start;
    public float3 End;
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
                rope.Start, rope.End
            );
            job.Run(job.RingCount);

            // generate indices
            var indices = new NativeArray<int>(job.IndexCount + job.IndexCountCaps, Allocator.TempJob);
            for (int i = 0; i < job.RingCount-1; i++)
            {
                int ringStart = i * job.VerticesPerRing;
                int nextRingStart = (i + 1) * job.VerticesPerRing;
                for (int j = 0; j < job.VerticesPerRing; j++)
                {
                    int nextJ = (j + 1) % job.VerticesPerRing;
                    int index = i * job.VerticesPerRing * 6 + j * 6;
                    indices[index] = ringStart + j;
                    indices[index + 1] = ringStart + nextJ;
                    indices[index + 2] = nextRingStart + j;
                    indices[index + 3] = nextRingStart + j;
                    indices[index + 4] = ringStart + nextJ;
                    indices[index + 5] = nextRingStart + nextJ;
                }
            }
            // start cap
            for (int i = 0; i < job.VerticesPerRing-1; i++)
            {
                int nextI = (i + 1) % job.VerticesPerRing;
                int index = job.IndexCount + i * 3;
                indices[index] = 0;
                indices[index + 1] = nextI;
                indices[index + 2] = i;
            }
            // end cap
            for (int i = 0; i < job.VerticesPerRing-1; i++)
            {
                int nextI = (i + 1) % job.VerticesPerRing;
                int index = job.IndexCount + job.IndexCountCaps/2 + i * 3;
                indices[index] = job.Vertices.Length - 1;
                indices[index + 1] = job.Vertices.Length - 1 - nextI;
                indices[index + 2] = job.Vertices.Length - 1 - i;
            }

            // create bone weights
            var boneWeights = new NativeArray<BoneWeight>(job.Vertices.Length, Allocator.TempJob);
            for (int i = 0; i < job.Vertices.Length; i++)
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
            for (int i = 0; i < bones.Length; i++)
            {
                var bone = new GameObject("Bone " + i);
                bone.transform.position = math.lerp(rope.Start, rope.End, (float)i/bones.Length);
                bones[i] = bone.transform;
                //Debug.DrawLine(bone.transform.position, bone.transform.position + Vector3.up, Color.red, 10f);
            }

            // create bind poses
            var bindPoses = new NativeArray<Matrix4x4>(bones.Length, Allocator.TempJob);
            for (int i = 0; i < bones.Length; i++)
                bindPoses[i] = bones[i].worldToLocalMatrix;

            // create bone entities
            var boneEntities = new NativeArray<Entity>(bones.Length, Allocator.TempJob);
            var boneArchetypeComponentList = new FixedList128Bytes<ComponentType> {
                ComponentType.ReadOnly<LocalToWorld>(), 
                ComponentType.ReadOnly<LocalTransform>(),
            }.ToNativeArray(state.WorldUpdateAllocator);
            var boneArchetype = state.EntityManager.CreateArchetype(boneArchetypeComponentList);

            for (int i = 0; i < bones.Length; i++){
                boneEntities[i] = state.EntityManager.CreateEntity(boneArchetype);
                state.EntityManager.SetComponentData(boneEntities[i], new LocalToWorld{Value = bones[i].localToWorldMatrix});
                state.EntityManager.SetName(boneEntities[i], bones[i].name);
                state.EntityManager.SetComponentData(boneEntities[i], LocalTransform.FromMatrix(bones[i].localToWorldMatrix));
                ecb.AddComponent(boneEntities[i], new CompanionLink{Companion = bones[i].gameObject});
            }

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
            // var mf = go.AddComponent<MeshFilter>();
            // mf.mesh = mesh;
            var mr = go.AddComponent<SkinnedMeshRenderer>();
            // get urp material
            var urp = UnityEngine.Rendering.Universal.UniversalRenderPipeline.asset;
            mr.material = urp.defaultMaterial;
            mr.sharedMesh = mesh;
            mr.material.color = new Color(0.5f, 0.3f, 0.1f);
            mr.bones = bones;

            // draw all verts
            // for (int i = 0; i < job.Vertices.Length; i++)
            // {
            //     Debug.DrawLine(job.Vertices[i], job.Vertices[i] + math.up() * 0.14f, Color.red, 20f);
            // }
            //job.Vertices.Dispose();
            indices.Dispose();
            job.Vertices.Dispose();
            boneWeights.Dispose();
            bindPoses.Dispose();
            Debug.Log("Rope vertices generated");
        }
    }
}

// Create cylinder verts job
[Unity.Burst.BurstCompile]
struct GenerateRopeVerticesJob : IJobFor {
    public readonly int SegmentCount; // number of segments
    public readonly int RingCount => SegmentCount * 2;
    readonly int verticesPerRing;
    public readonly int VerticesPerRing => verticesPerRing;
    
    readonly float middleRadius;
    readonly float capRadius;
    readonly float3 start;
    readonly float3 end;

    readonly float deltaOut;

    [NativeDisableParallelForRestriction] [WriteOnly] 
    public NativeArray<float3> vertices; // we know the size is RingCount * CircleVertexCount


    public readonly NativeArray<float3> Vertices => vertices;
    public readonly int VertexCount => vertices.Length;

    public readonly int IndexCount => RingCount * VerticesPerRing * 6;
    public readonly int IndexCountCaps => (VerticesPerRing-1) * 2 * 3;

    public GenerateRopeVerticesJob(float3 start, float3 end, int segmentCount = 4, int verticesPerRing = 5, float middleRadius = 0.1f, float capRadius = 0.05f, float deltaOut = 0.1f) {
        this.start = start;
        this.end = end;
        this.SegmentCount = segmentCount;
        this.verticesPerRing = verticesPerRing;
        this.middleRadius = middleRadius;
        this.capRadius = capRadius;
        this.deltaOut = deltaOut;
        var ringCount = SegmentCount * 2;
        this.vertices = new NativeArray<float3>(ringCount * verticesPerRing, Allocator.TempJob);
    }

    // 0 is start ring, SegmentCount*2 is end ring
    public void Execute(int ringIndex) {
        // create end and start ring
        if (ringIndex == 0 || ringIndex == RingCount-1) {
            // create vertices
            var radius = capRadius;
            var startIndex = ringIndex * VerticesPerRing;
            var direction = end-start;
            var isFirst = ringIndex % 2 == 0;
            var delta = isFirst ? -deltaOut : deltaOut;
            var pushOut = math.normalize(direction) * delta;
            var startPos = (ringIndex == 0 ? start : end) + pushOut;

            // create ring
            for (int i = 0; i < VerticesPerRing; i++) {
                var angle = (float)i / VerticesPerRing * math.PI * 2;
                var x = math.cos(angle) * radius;
                var y = math.sin(angle) * radius;
                var pos = new float3(x, y, 0);
                vertices[startIndex + i] = math.mul(quaternion.LookRotationSafe(direction, math.up()), pos) + startPos;
            }
        } else {
            // create vertices
            var radius = middleRadius;
            var startIndex = ringIndex * VerticesPerRing;
            var direction = end-start;
            var isFirst = ringIndex % 2 == 0;
            var delta = isFirst ? -deltaOut : deltaOut;
            var pushOut = math.normalize(direction) * delta;
            var offset = math.lerp(start, end, (float)(ringIndex/2) / (SegmentCount-1));
            var startPos = offset + pushOut;

            // create ring
            for (int i = 0; i < VerticesPerRing; i++) {
                var angle = (float)i / VerticesPerRing * math.PI * 2;
                var x = math.cos(angle) * radius;
                var y = math.sin(angle) * radius;
                var pos = new float3(x, y, 0);
                vertices[startIndex + i] = math.mul(quaternion.LookRotationSafe(direction, math.up()), pos) + startPos;
            }
        }
    }
}