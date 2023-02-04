using Unity.Entities;
using UnityEngine;

public partial class CameraSystem : SystemBase
{
    bool shouldRun = true;
    protected override void OnUpdate()
    {
        if (!shouldRun)
            return;

        var ecb = SystemAPI.GetSingleton<EndInitializationEntityCommandBufferSystem.Singleton>().CreateCommandBuffer(World.Unmanaged);
        foreach (var player in SystemAPI.Query<ThirdPersonPlayer>().WithAll<Simulate>())
        {            
            if (!SystemAPI.ManagedAPI.HasComponent<CompanionLink>(player.ControlledCamera))
            {
                ecb.AddComponent(player.ControlledCamera, new CompanionLink{Companion=Camera.main.gameObject});
                Cursor.lockState = CursorLockMode.Locked;
                shouldRun = false;
            }
        }
    }
}