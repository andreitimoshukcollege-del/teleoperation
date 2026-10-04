using UnityEngine;
using UnityEngine.XR.Interaction.Toolkit.Interactables;

// C# 9: block-scoped namespace only, like the rest of unity/.
namespace Teleop.XR
{
    /// <summary>
    /// Colours the arm's target by interaction state: idle, hovered (a pinch now would grab it) and
    /// grabbed. With hands there is no trigger to feel. Seeing "hovered" before you pinch is how you
    /// know the pinch lands on the target and not on empty air, and seeing "grabbed" is how you know the
    /// robot is now following your hand.
    ///
    /// <para>Display only: reads the interactable's state and recolours one renderer through a
    /// property block (both <c>_BaseColor</c> and <c>_Color</c>, as Bridge does, so it works in either
    /// pipeline). No allocation per frame.</para>
    /// </summary>
    [RequireComponent(typeof(XRBaseInteractable))]
    public sealed class GrabHighlight : MonoBehaviour
    {
        private static readonly int BaseColorId = Shader.PropertyToID("_BaseColor");
        private static readonly int ColorId = Shader.PropertyToID("_Color");

        [SerializeField] private Renderer target;
        [SerializeField] private Color idleColor = new Color(0.1f, 0.75f, 0.95f);
        [SerializeField] private Color hoverColor = new Color(1f, 0.85f, 0.2f);
        [SerializeField] private Color grabbedColor = new Color(0.25f, 0.95f, 0.4f);

        private XRBaseInteractable _interactable;
        private MaterialPropertyBlock _block;
        private int _state = -1;

        private void Awake()
        {
            _interactable = GetComponent<XRBaseInteractable>();
            if (target == null)
            {
                target = GetComponent<Renderer>();
            }

            _block = new MaterialPropertyBlock();
        }

        private void Update()
        {
            int state = _interactable.isSelected ? 2 : _interactable.isHovered ? 1 : 0;
            if (state == _state || target == null)
            {
                return;
            }

            _state = state;
            Color color = state == 2 ? grabbedColor : state == 1 ? hoverColor : idleColor;
            target.GetPropertyBlock(_block);
            _block.SetColor(BaseColorId, color);
            _block.SetColor(ColorId, color);
            target.SetPropertyBlock(_block);
        }
    }
}
