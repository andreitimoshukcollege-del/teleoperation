using UnityEngine;

// C# 9: block-scoped namespace only, like the rest of unity/.
namespace Teleop.XR
{
    /// <summary>
    /// Draws one of the twin's arms in a see-through material (docs/adr/0018): the commanded arm as a
    /// ghost, the last-measured arm faint. At <c>Awake</c> every renderer under this object takes
    /// <see cref="material"/>, except those in <see cref="keep"/> (the reach warning has to stay
    /// red). The scene keeps the solid materials, so previews and the editor show every arm solid;
    /// only the running app is see-through.
    /// </summary>
    public sealed class ArmAppearance : MonoBehaviour
    {
        [SerializeField] private Material material;
        [SerializeField] private Renderer[] keep = new Renderer[0];

        private void Awake()
        {
            if (material == null)
            {
                return;
            }

            foreach (Renderer r in GetComponentsInChildren<Renderer>(true))
            {
                if (System.Array.IndexOf(keep, r) >= 0)
                {
                    continue;
                }

                var materials = new Material[r.sharedMaterials.Length];
                for (int i = 0; i < materials.Length; i++) materials[i] = material;
                r.sharedMaterials = materials;
                r.shadowCastingMode = UnityEngine.Rendering.ShadowCastingMode.Off;
            }
        }
    }
}
