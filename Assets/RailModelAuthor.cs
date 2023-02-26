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

                    // add square vertices
                    var face0 = toStartPoint + toStartPointRight * width + up * height;
                    var face1 = toStartPoint + toStartPointRight * width - up * height;
                    var face2 = toStartPoint - toStartPointRight * width - up * height;
                    var face3 = toStartPoint - toStartPointRight * width + up * height;

                    // overall rotation
                    var fullRotation = Vector3.SignedAngle(toStartPoint - pivot, toEndPoint - pivot, up);
                    fullRotation = math.radians(fullRotation);
                    var halfRotation = fullRotation / 2f;
                    var rotationStep = halfRotation / cornerSegments;
                    var offsetRotation = isStart ? halfRotation : 0;
                    // rotate around pivot to get face 4, 5, 6, 7
                    for (var i = 0; i <= cornerSegments; i++) {
                        var rotation = quaternion.AxisAngle(up, i * rotationStep + offsetRotation);

                        vertices.Add(math.mul(rotation, face0 - pivot) + pivot);
                        vertices.Add(math.mul(rotation, face1 - pivot) + pivot);
                        vertices.Add(math.mul(rotation, face2 - pivot) + pivot);
                        vertices.Add(math.mul(rotation, face3 - pivot) + pivot);
                        normals.AddReplicate(up, 4);
                        uvs.Add(new float2(0, 0));
                        uvs.Add(new float2(0, 1));
                        uvs.Add(new float2(1, 1));
                        uvs.Add(new float2(1, 0));
                    }

                    Debug.DrawLine(toStartPoint, toStartPoint + toStartPointRight, Color.green, 2f);
                    Debug.DrawLine(toEndPoint, toEndPoint + toEndPointRight, Color.red, 2f);
                    Debug.DrawLine(middlePoint, middlePoint + (float3)toStartPlane.normal*1f, Color.green, 2f);
                    Debug.DrawLine(middlePoint, middlePoint + (float3)toEndPlane.normal*1f, Color.red, 2f);
                    Debug.DrawLine(middlePoint+toStartDirectionWithMagnitude, pivot, Color.blue, 2f);
                    
                    // draw planes (planeBetweenStartAndP0 and planeBetweenP0AndP3)
                    DebugDrawPlane(middlePoint+toStartDirectionWithMagnitude, toStartPlane, up, 1, Color.green);
                    DebugDrawPlane(middlePoint+toEndDirectionWithMagnitude, toEndPlane, up, 1, Color.red);
                    Debug.DrawLine(toStartPoint, toStartPoint + up, Color.green, 2f);
                    Debug.DrawLine(toEndPoint, toEndPoint + up, Color.red, 2f);
                }
                
                void AddStub(bool isStart) {
                    var forward = curveP3 - curveP0;
                    forward = math.normalize(forward);
                    var knot = spline[curveIndex + (isStart ? 0 : 1)];
                    var up = math.mul(knot.Rotation, math.up());
                    var right = math.cross(forward, up);
                    right = math.normalize(right);
                    var rightOffset = right * width;
                    var upOffset = up * height;

                    var point = transform.TransformPoint(isStart ? curveP0 : curveP3);
                    vertices.Add(point + rightOffset + upOffset);
                    vertices.Add(point + rightOffset - upOffset);
                    vertices.Add(point - rightOffset - upOffset);
                    vertices.Add(point - rightOffset + upOffset);

                    normals.AddReplicate(up, 4);
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


                // for (var t = 0f; t <= 1f; t += 0.01f) {
                //     // if middle then early out
                //     if (t is > 0.1f and < 0.9f)
                //         continue;
                //
                //
                //     var splineT = spline.CurveToSplineT(curveIndex+spline.GetCurveInterpolation(curveIndex, t*spline.GetCurveLength(curveIndex)));
                //     spline.Evaluate(splineT, out var position, out var tangent, out var normal);
                //     // draw point
                //     Debug.DrawLine(position, position + normal * 0.1f, Color.green, 2);
                //     Debug.DrawLine(position, position + tangent * 0.1f, Color.blue, 2);
                //
                //     // set up quad
                //     var bitangent = math.cross(tangent, normal);
                //     var right = math.normalize(bitangent) * width;
                //     var up = normal * height;
                //     
                //     vertices.Add(position + right + up);
                //     vertices.Add(position + right - up);
                //     vertices.Add(position - right - up);
                //     vertices.Add(position - right + up);
                //     
                //     normals.AddReplicate(normal, 4);
                //
                //     uvs.Add(new float2(0, 0));
                //     uvs.Add(new float2(0, 1));
                //     uvs.Add(new float2(1, 1));
                //     uvs.Add(new float2(1, 0));
                // }
            }

            // draw debug
            for (var i = 0; i < vertices.Length; i++) { 
                Debug.DrawLine(vertices[i], vertices[i] + normals[i] * 0.05f, Color.green, 2);
            }
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