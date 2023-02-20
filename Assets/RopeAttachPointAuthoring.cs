using Unity.Entities;
using UnityEngine;

public class RopeAttachPointAuthoring : MonoBehaviour {
    public class RopeAttachPointBaker : Baker<RopeAttachPointAuthoring> {
        public override void Bake(RopeAttachPointAuthoring authoring) {
            AddComponent(new RopeAttachPoint());
        }
    }
}

struct RopeAttachPoint : IComponentData {}