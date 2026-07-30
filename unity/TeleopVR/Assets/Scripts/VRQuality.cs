using UnityEngine;
using UnityEngine.XR;

public class VRQuality : MonoBehaviour
{
    [Range(0.5f, 2.0f)]
    public float resolutionScale = 1.3f;

    void Start()
    {
        XRSettings.eyeTextureResolutionScale = resolutionScale;
        Debug.Log($"Eye texture resolution scale set to {XRSettings.eyeTextureResolutionScale}");
    }
}