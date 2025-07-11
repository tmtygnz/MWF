using Godot;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace MiniWarfare.Game.Scripts.Utils
{
    public static class VectorUtils
    {
        public static Vector3 ScaleEach(Vector3 input, float XPos, float XNeg, float YPos, float YNeg, float ZPos, float ZNeg)
        {
            float X = input.X >= 0 ? input.X * XPos : input.X * XNeg;
            float Y = input.Y >= 0 ? input.Y * YPos : input.Y * YNeg;
            float Z = input.Z >= 0 ? input.Z * ZPos : input.Z * ZNeg;
            return new Vector3(X, Y, Z);
        }
    }
}
