using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;
using Unity.Rendering;
using Unity.Transforms;
using UnityEngine;
using UnityEngine.Splines;

[RequireComponent(typeof(SplineContainer))]
public class RailModelAuthor : MonoBehaviour {
    public float height = 1;
    public float width = 1;
    public float cornerRadius = 0.05f;
    public int cornerSegments = 8;
    public Material material;
    
    class Baker : Baker<RailModelAuthor> {
        public override void Bake(RailModelAuthor auth) {
            var spline = GetComponent<SplineContainer>(auth);
            DependsOn(auth.transform);
            AddComponent(new RailSpline {spline = new NativeSpline(spline.Spline, Allocator.Persistent)});
            AddComponent(new RailModelInfo {
                height = auth.height, width = auth.width, 
                cornerRadius = auth.cornerRadius, cornerSegments = auth.cornerSegments
            });
            AddComponentObject(new RailModel {material = auth.material});
        }
    }
}

struct RailModelInfo : IComponentData {
    public float height;
    public float width;
    public float cornerRadius;
    public int cornerSegments;
}

[TemporaryBakingType]
struct RailSpline : IComponentData {
    public NativeSpline spline;
}

[BakingType]
class RailModel : IComponentData {
    public Mesh mesh;
    public Material material;
}

// Create system to bake the rail mesh
// mesh is created from a spline
// the sampling is non-uniform, with more samples at Knots

[WorldSystemFilter(WorldSystemFilterFlags.BakingSystem)]
partial struct RailMeshBakingSystem : ISystem {
    public void OnUpdate(ref SystemState state) {
        foreach (var (railSpline, transform, railModelInfo) in SystemAPI
                     .Query<RailSpline, LocalTransform, RailModelInfo>()) {
            var spline = railSpline.spline;
            var height = railModelInfo.height;
            var width = railModelInfo.width;
            var cornerRadius = railModelInfo.cornerRadius;
            var cornerSegments = railModelInfo.cornerSegments;
            var metersPerSegment = 0.1f;

            var vertices = new NativeList<float3>(Allocator.Temp);
            var indices = new NativeList<int>(Allocator.Temp);
            var normals = new NativeList<float3>(Allocator.Temp);
            var uvs = new NativeList<float2>(Allocator.Temp);

            // Add vertices
            var curveCount = spline.GetCurveCount();
            for (var curveIndex = 0; curveIndex < curveCount; curveIndex++) {
                var curve = spline.GetCurve(curveIndex);
                var curveP0 = curve.P0;
                var curveP3 = curve.P3;

                void AddHalfCurve(bool isStart) {
                    // create square
                    var startPoint = isStart ? spline[curveIndex - 1].Position : curveP0;
                    var middlePoint = isStart ? curveP0 : curveP3;
                    var endPoint = isStart ? curveP3 : spline[curveIndex + 2].Position;
                    startPoint = transform.TransformPoint(startPoint);
                    middlePoint = transform.TransformPoint(middlePoint);
                    endPoint = transform.TransformPoint(endPoint);

                    var knot = spline[curveIndex + (isStart ? 0 : 1)];
                    var up = math.mul(knot.Rotation, math.up());
                    
                    var middlePointToStart = startPoint - middlePoint;
                    var toStartDirection = math.normalize(middlePointToStart);
                    var toStartDirectionWithMagnitude = toStartDirection * cornerRadius;
                    var toStartPoint = middlePoint + toStartDirectionWithMagnitude;
                    var toStartPointRight = math.cross(toStartDirection, up);
                    var toStartPlane = new Plane(toStartDirection, toStartPoint);

                    var middlePointToEnd = endPoint - middlePoint;
                    var toEndDirection = math.normalize(middlePointToEnd);
                    var toEndDirectionWithMagnitude = toEndDirection * cornerRadius;
                    var toEndPoint = middlePoint + toEndDirectionWithMagnitude;
                    var toEndPointRight = math.cross(toEndDirection, up);
                    var toEndPlane = new Plane(toEndDirection, toEndPoint);

                    // get intersection between planes
                    var ray = new Ray(toStartPoint, toStartPointRight);
                    toEndPlane.Raycast(ray, out var distanceAlongRay);
                    var pivot = (float3)ray.GetPoint(distanceAlongRay);

                    // face verts
                    var faceP0 = toStartPoint + toStartPointRight * width + up * height;
                    var faceP1 = toStartPoint + toStartPointRight * width - up * height;
                    var faceP2 = toStartPoint - toStartPointRight * width - up * height;
                    var faceP3 = toStartPoint - toStartPointRight * width + up * height;
                    
                    // face normals
                    var faceN00 = up;
                    var faceN01 = toStartPointRight;
                    
                    var faceN10 = toStartPointRight;
                    var faceN11 = -up;
                    
                    var faceN20 = -up;
                    var faceN21 = -toStartPointRight;
                    
                    var faceN30 = -toStartPointRight;
                    var faceN31 = up;

                    // overall rotation
                    var fullRotation = Vector3.SignedAngle(toStartPoint - pivot, toEndPoint - pivot, up);
                    fullRotation = math.radians(fullRotation);
                    var halfRotation = fullRotation / 2f;
                    var rotationStep = halfRotation / cornerSegments;
                    var offsetRotation = isStart ? halfRotation : 0;
                    // rotate around pivot to get face 4, 5, 6, 7
                    for (var i = 0; i <= cornerSegments; i++) {
                        var rotation = quaternion.AxisAngle(up, i * rotationStep + offsetRotation);

                        vertices.AddReplicate(math.mul(rotation, faceP0 - pivot) + pivot, 2);
                        vertices.AddReplicate(math.mul(rotation, faceP1 - pivot) + pivot, 2);
                        vertices.AddReplicate(math.mul(rotation, faceP2 - pivot) + pivot, 2);
                        vertices.AddReplicate(math.mul(rotation, faceP3 - pivot) + pivot, 2);
                        
                        // add normals
                        normals.Add(math.mul(rotation, faceN00));
                        normals.Add(math.mul(rotation, faceN01));
                        
                        normals.Add(math.mul(rotation, faceN10));
                        normals.Add(math.mul(rotation, faceN11));
                        
                        normals.Add(math.mul(rotation, faceN20));
                        normals.Add(math.mul(rotation, faceN21));
                        
                        normals.Add(math.mul(rotation, faceN30));
                        normals.Add(math.mul(rotation, faceN31));

                        uvs.AddReplicate(new float2(0, 0), 2);
                        uvs.AddReplicate(new float2(0, 1), 2);
                        uvs.AddReplicate(new float2(1, 1), 2);
                        uvs.AddReplicate(new float2(1, 0), 2);
                        
                        // add indices connecting to previous vertices flat shaded
                        var indexOffset = vertices.Length-16;
                        if (indexOffset < 0) continue;
                        indices.Add(indexOffset + 2);
                        indices.Add(indexOffset + 1);
                        indices.Add(indexOffset + 8 + 1);
                        indices.Add(indexOffset + 2);
                        indices.Add(indexOffset + 8 + 1);
                        indices.Add(indexOffset + 8 + 2);
                        
                        indices.Add(indexOffset + 4);
                        indices.Add(indexOffset + 3);
                        indices.Add(indexOffset + 8 + 3);
                        indices.Add(indexOffset + 4);
                        indices.Add(indexOffset + 8 + 3);
                        indices.Add(indexOffset + 8 + 4);
                        
                        indices.Add(indexOffset + 6);
                        indices.Add(indexOffset + 5);
                        indices.Add(indexOffset + 8 + 5);
                        indices.Add(indexOffset + 6);
                        indices.Add(indexOffset + 8 + 5);
                        indices.Add(indexOffset + 8 + 6);
                        
                        indices.Add(indexOffset + 8);
                        indices.Add(indexOffset + 7);
                        indices.Add(indexOffset + 8 + 7);
                        indices.Add(indexOffset);
                        indices.Add(indexOffset + 7);
                        indices.Add(indexOffset + 8);
                    }

                    // Debug.DrawLine(toStartPoint, toStartPoint + toStartPointRight, Color.green, 2f);
                    // Debug.DrawLine(toEndPoint, toEndPoint + toEndPointRight, Color.red, 2f);
                    // Debug.DrawLine(middlePoint, middlePoint + (float3)toStartPlane.normal*1f, Color.green, 2f);
                    // Debug.DrawLine(middlePoint, middlePoint + (float3)toEndPlane.normal*1f, Color.red, 2f);
                    // Debug.DrawLine(middlePoint+toStartDirectionWithMagnitude, pivot, Color.blue, 2f);
                    //
                    // // draw planes (planeBetweenStartAndP0 and planeBetweenP0AndP3)
                    // DebugDrawPlane(middlePoint+toStartDirectionWithMagnitude, toStartPlane, up, 1, Color.green);
                    // DebugDrawPlane(middlePoint+toEndDirectionWithMagnitude, toEndPlane, up, 1, Color.red);
                    // Debug.DrawLine(toStartPoint, toStartPoint + up, Color.green, 2f);
                    // Debug.DrawLine(toEndPoint, toEndPoint + up, Color.red, 2f);
                }
                
                void AddStub(bool isStart) {
                    var forward = curveP0-curveP3;
                    forward = math.normalize(forward);
                    var knot = spline[curveIndex + (isStart ? 0 : 1)];
                    var up = math.mul(knot.Rotation, math.up());
                    var right = math.cross(forward, up);
                    right = math.normalize(right);
                    var rightOffset = right * width;
                    var upOffset = up * height;

                    var point = transform.TransformPoint(isStart ? curveP0 : curveP3);
                    var faceP0 = point + rightOffset + upOffset;
                    var faceP1 = point + rightOffset - upOffset;
                    var faceP2 = point - rightOffset - upOffset;
                    var faceP3 = point - rightOffset + upOffset;
                    
                    var faceN00 = up;
                    var faceN01 = right;
                    
                    var faceN10 = right;
                    var faceN11 = -up;
                    
                    var faceN20 = -up;
                    var faceN21 = -right;
                    
                    var faceN30 = -right;
                    var faceN31 = up;

                    vertices.AddReplicate(faceP0, 2);
                    vertices.AddReplicate(faceP1, 2);
                    vertices.AddReplicate(faceP2, 2);
                    vertices.AddReplicate(faceP3, 2);
                    
                    normals.Add(faceN00);
                    normals.Add(faceN01);
                    normals.Add(faceN10);
                    normals.Add(faceN11);
                    normals.Add(faceN20);
                    normals.Add(faceN21);
                    normals.Add(faceN30);
                    normals.Add(faceN31);
                    
                    uvs.AddReplicate(new float2(0, 0),2);
                    uvs.AddReplicate(new float2(0, 1),2);
                    uvs.AddReplicate(new float2(1, 1),2);
                    uvs.AddReplicate(new float2(1, 0),2);

                    // add indices
                    var indexOffset = vertices.Length-16;
                    if (indexOffset < 0) return;
                    indices.Add(indexOffset + 2);
                    indices.Add(indexOffset + 1);
                    indices.Add(indexOffset + 8 + 1);
                    indices.Add(indexOffset + 2);
                    indices.Add(indexOffset + 8 + 1);
                    indices.Add(indexOffset + 8 + 2);
                    
                    indices.Add(indexOffset + 4);
                    indices.Add(indexOffset + 3);
                    indices.Add(indexOffset + 8 + 3);
                    indices.Add(indexOffset + 4);
                    indices.Add(indexOffset + 8 + 3);
                    indices.Add(indexOffset + 8 + 4);
                    
                    indices.Add(indexOffset + 6);
                    indices.Add(indexOffset + 5);
                    indices.Add(indexOffset + 8 + 5);
                    indices.Add(indexOffset + 6);
                    indices.Add(indexOffset + 8 + 5);
                    indices.Add(indexOffset + 8 + 6);
                    
                    indices.Add(indexOffset + 8);
                    indices.Add(indexOffset + 7);
                    indices.Add(indexOffset + 8 + 7);
                    indices.Add(indexOffset);
                    indices.Add(indexOffset + 7);
                    indices.Add(indexOffset + 8);
                }

                // should create start curve
                if (!spline.Closed && curveIndex == 0) {
                    // *---------------*
                    // |  S*---x       |
                    // |       |       |
                    // |       |       |
                    // |       x----E  |
                    // *---------------*
                    AddStub(true);
                } else {
                    // *---------------*
                    // |  S----x       |
                    // |       *       |
                    // |       |       |
                    // |       x*---E  |
                    // *---------------*
                    AddHalfCurve(true);
                }

                // should create end curve
                if (!spline.Closed && curveIndex == curveCount - 1) {
                    // *---------------*
                    // |  S----x       |
                    // |       |       |
                    // |       |       |
                    // |       x---*E  |
                    // *---------------*
                    AddStub(false);
                } else {
                    // *---------------*
                    // |  S---*x       |
                    // |       |       |
                    // |       *       |
                    // |       x----E  |
                    // *---------------*
                    AddHalfCurve(false);
                }
            }
            
            // add indices for end caps
            var offset = vertices.Length;
            vertices.Add(vertices[offset-7]);
            vertices.Add(vertices[offset-5]);
            vertices.Add(vertices[offset-3]);
            vertices.Add(vertices[offset-1]);
            normals.AddReplicate(math.normalize(spline[^1].Position - spline[^2].Position), 4);
            uvs.Add(new float2(0, 0));
            uvs.Add(new float2(0, 1));
            uvs.Add(new float2(1, 1));
            uvs.Add(new float2(1, 0));
            
            indices.Add(vertices.Length-1);
            indices.Add(vertices.Length-2);
            indices.Add(vertices.Length-3);
            indices.Add(vertices.Length-3);
            indices.Add(vertices.Length-4);
            indices.Add(vertices.Length-1);
            
            vertices.Add(vertices[0]);
            vertices.Add(vertices[2]);
            vertices.Add(vertices[4]);
            vertices.Add(vertices[6]);
            normals.AddReplicate(math.normalize(spline[0].Position - spline[1].Position), 4);
            uvs.Add(new float2(0, 0));
            uvs.Add(new float2(0, 1));
            uvs.Add(new float2(1, 1));
            uvs.Add(new float2(1, 0));

            indices.Add(vertices.Length-4);
            indices.Add(vertices.Length-3);
            indices.Add(vertices.Length-2);
            indices.Add(vertices.Length-2);
            indices.Add(vertices.Length-1);
            indices.Add(vertices.Length-4);
            
            // debug draw normals
            for (int i = 0; i < vertices.Length; i++) {
                Debug.DrawLine(vertices[i], vertices[i] + normals[i] * 0.1f, Color.red, 2f);
            }
            
            // set mesh data
            var mesh = new Mesh();
            mesh.SetVertices(vertices.AsArray());
            mesh.SetNormals(normals.AsArray());
            mesh.SetUVs(0, uvs.AsArray());
            mesh.SetIndices(indices.ToArray(Allocator.Temp), MeshTopology.Triangles, 0);
            mesh.RecalculateBounds();
            
            // get urp default material
            var material = new Material(Shader.Find("Universal Render Pipeline/Lit"));

            // Create a GameObject with the mesh;
            var go = new GameObject("SplineMesh");
            go.AddComponent<MeshFilter>().sharedMesh = mesh;
            go.AddComponent<MeshRenderer>().sharedMaterial = material;
            material.color = new Color(0.65f, 0.5f, 0.6f);
        }
    }
    
    // debug draw plane
    static void DebugDrawPlane(float3 point, Plane plane, float3 up, float squareSize, Color color, float duration = 2f) {
        // draw square
        var pointOnPlane = (float3)plane.ClosestPointOnPlane(point);
        var right = math.cross(plane.normal, up);
        var p0 = pointOnPlane + right * squareSize + up * squareSize;
        var p1 = pointOnPlane + right * squareSize - up * squareSize;
        var p2 = pointOnPlane - right * squareSize - up * squareSize;
        var p3 = pointOnPlane - right * squareSize + up * squareSize;
        Debug.DrawLine(p0, p1, color, duration);
        Debug.DrawLine(p1, p2, color, duration);
        Debug.DrawLine(p2, p3, color, duration);
        Debug.DrawLine(p3, p0, color, duration);
    }
}

[WorldSystemFilter(WorldSystemFilterFlags.BakingSystem)]
partial struct RailModelRenderArrayBakingSystem : ISystem {
    public void OnUpdate(ref SystemState state) {
        
    }
}