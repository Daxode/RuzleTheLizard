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

                // should create start curve
                if (curveIndex == 0 && spline.Closed || curveIndex > 0) {
                    // *---------------*
                    // |  S----x       |
                    // |       *       |
                    // |       |       |
                    // |       x*---E  |
                    // *---------------*
                    // create square
                    var p0 = transform.TransformPoint(curve.P0);
                    var p3 = transform.TransformPoint(curve.P3);
                    
                    // var forward = curve.P3 - curve.P0;
                    // forward = math.normalize(forward);
                    var knot = spline[curveIndex];
                    var up = math.mul(knot.Rotation, math.up());
                    // var right = math.cross(forward, up);
                    // right = math.normalize(right);

                    var previousKnot = spline[curveIndex-1];
                    var p0ToPreviousKnot = transform.TransformPoint(previousKnot.Position) - p0;
                    
                    var p0ToPreviousKnotDirection = math.normalize(p0ToPreviousKnot);
                    var circleDirectionToPreviousKnot = p0ToPreviousKnotDirection*cornerRadius;
                    var toPreviousKnotPoint = p0 + circleDirectionToPreviousKnot;
                    var planeBetweenPreviousKnotAndP0 = new Plane(p0ToPreviousKnotDirection, toPreviousKnotPoint);

                    var p0ToP3 = p3 - p0;
                    var p0ToP3Direction = math.normalize(p0ToP3);
                    var circleDirectionToP3 = p0ToP3Direction*cornerRadius;
                    var toP3Point = p0 + circleDirectionToP3;
                    var planeBetweenP3AndP0 = new Plane(p0ToP3Direction, toP3Point);

                    // draw planes (planeBetweenPreviousKnotAndP0 and planeBetweenP0AndP3)
                    DebugDrawPlane(p0+circleDirectionToPreviousKnot, planeBetweenPreviousKnotAndP0, up, 1, Color.green);
                    DebugDrawPlane(p0+circleDirectionToP3, planeBetweenP3AndP0, up, 1, Color.red);
                    
                    // draw plane up
                    Debug.DrawLine(p0+circleDirectionToPreviousKnot, p0+circleDirectionToPreviousKnot + up, Color.green, 2f);
                    Debug.DrawLine(p0+circleDirectionToP3, p0+circleDirectionToP3 + up, Color.red, 2f);
                    
                    // draw plane right
                    var rightP0ToPreviousKnot = math.cross(p0ToPreviousKnotDirection, up);
                    Debug.DrawLine(p0+circleDirectionToPreviousKnot, p0+circleDirectionToPreviousKnot + rightP0ToPreviousKnot, Color.green, 2f);
                    var rightP0ToP3 = math.cross(p0ToP3Direction, up);
                    Debug.DrawLine(p0+circleDirectionToP3, p0+circleDirectionToP3 + rightP0ToP3, Color.red, 2f);

                    // draw normals
                    Debug.DrawLine(p0, p0 + (float3)planeBetweenPreviousKnotAndP0.normal*1f, Color.green, 2f);
                    Debug.DrawLine(p0, p0 + (float3)planeBetweenP3AndP0.normal*1f, Color.red, 2f);
                    
                    // get intersection between planes
                    var ray = new Ray(p0+circleDirectionToPreviousKnot, rightP0ToPreviousKnot);
                    planeBetweenP3AndP0.Raycast(ray, out var distanceAlongRay);
                    var pivot = (float3)ray.GetPoint(distanceAlongRay);
                    Debug.DrawLine(p0+circleDirectionToPreviousKnot, pivot, Color.blue, 2f);

                    // add square vertices
                    var face0 = toPreviousKnotPoint + rightP0ToPreviousKnot * width + up * height;
                    var face1 = toPreviousKnotPoint + rightP0ToPreviousKnot * width - up * height;
                    var face2 = toPreviousKnotPoint - rightP0ToPreviousKnot * width - up * height;
                    var face3 = toPreviousKnotPoint - rightP0ToPreviousKnot * width + up * height;

                    // overall rotation
                    var fullRotation = Vector3.SignedAngle(toPreviousKnotPoint-pivot, toP3Point-pivot, up);
                    fullRotation = math.radians(fullRotation);
                    var halfRotation = fullRotation/2f;
                    var rotationStep = halfRotation/cornerSegments;
                    // rotate around pivot to get face 4, 5, 6, 7
                    for (var i = 0; i <= cornerSegments; i++) {
                        var rotation = quaternion.AxisAngle(up, i*rotationStep+halfRotation);

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
                } else {
                    // *---------------*
                    // |  S*---x       |
                    // |       |       |
                    // |       |       |
                    // |       x----E  |
                    // *---------------*
                    // create square
                    Debug.Log("Creating start square");
                    var forward = curve.P3 - curve.P0;
                    forward = math.normalize(forward);
                    var knot = spline[curveIndex];
                    var up = math.mul(knot.Rotation, math.up());
                    var right = math.cross(forward, up);
                    right = math.normalize(right);
                    var rightOffset = right * width;
                    var upOffset = up * height;
                    var p0 = transform.TransformPoint(curve.P0);
                    
                    vertices.Add(p0 + rightOffset + upOffset);
                    vertices.Add(p0 + rightOffset - upOffset);
                    vertices.Add(p0 - rightOffset - upOffset);
                    vertices.Add(p0 - rightOffset + upOffset);
                    
                    normals.AddReplicate(up, 4);
                }
                
                // should create end curve
                if (curveIndex == curveCount - 1 && spline.Closed || curveIndex < curveCount - 1) {
                    // *---------------*
                    // |  S---*x       |
                    // |       |       |
                    // |       *       |
                    // |       x----E  |
                    // *---------------*
                    // create square
                } else {
                    // *---------------*
                    // |  S----x       |
                    // |       |       |
                    // |       |       |
                    // |       x---*E  |
                    // *---------------*
                    // create square
                    Debug.Log("Creating end square");
                    var forward = curve.P3 - curve.P0;
                    forward = math.normalize(forward);
                    var knot = spline[curveIndex+1];
                    var up = math.mul(knot.Rotation, math.up());
                    var right = math.cross(forward, up);
                    right = math.normalize(right);
                    var rightOffset = right * width;
                    var upOffset = up * height;
                    var p3 = transform.TransformPoint(curve.P3);
                    
                    vertices.Add(p3 + rightOffset + upOffset);
                    vertices.Add(p3 + rightOffset - upOffset);
                    vertices.Add(p3 - rightOffset - upOffset);
                    vertices.Add(p3 - rightOffset + upOffset);
                    
                    normals.AddReplicate(up, 4);
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
                Debug.DrawLine(vertices[i], vertices[i] + normals[i] * 0.1f, Color.green, 2);
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