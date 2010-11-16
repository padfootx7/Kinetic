using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Microsoft.Xna.Framework;

namespace Kinetic
{
    public class Matrix2
    {
        // Internal values [ M11, M21]
        //                 [ M12, M22]
        private float M11, M21, M12, M22;

        public Matrix2() { }

        public Matrix2(float angle)
        {
            float c = (float) Math.Cos(angle);
            float s = (float) Math.Sin(angle);

            // Create the matrix values
            M11 = c;
            M21 = -s;
            M12 = s;
            M22 = c;
        }

        public Matrix2(float a, float b, float c, float d)
        {
            M11 = a; M21 = b;
            M12 = c; M22 = d;
        }

        // Tranpose matrix values (switch rows and columns)
        public Matrix2 Transpose()
        {
            return new Matrix2(M11, M12, 
                               M21, M22); // Switch between M12 & M21
        }

        // Invert matrix values
        public Matrix2 Invert()
        {
            Matrix2 returnMatrix = new Matrix2();
            float det = M11 * M22 - M21 * M12;

            det = 1.0f / det;

            returnMatrix.M11 = det * M22;
            returnMatrix.M21 = -det * M21;
            returnMatrix.M12 = -det * M12;
            returnMatrix.M22 = det * M11;

            return returnMatrix;
        }

        // Multiply Matrices
        public static Matrix2 Multiply(Matrix2 A, Matrix2 B)
        {
            float m11 = A.M11 * B.M11 + A.M21 * B.M12;
            float m12 = A.M12 * B.M11 + A.M22 * B.M12;
            float m21 = A.M11 * B.M21 + A.M21 * B.M22;
            float m22 = A.M12 * B.M21 + A.M22 * B.M22;

            return new Matrix2(m11, m21, m12, m22);
        }

        public static Vector2 Multiply(Matrix2 A, Vector2 V)
        {
            float x = A.M11 * V.X + A.M21 * V.Y;
            float y = A.M12 * V.X + A.M22 * V.Y;

            return new Vector2(x, y);
        }

        public static Matrix2 Add(Matrix2 A, Matrix2 B)
        {
            float m11 = A.M11 + B.M11;
            float m12 = A.M12 + B.M12;
            float m21 = A.M21 + B.M21;
            float m22 = A.M22 + B.M22;

            return new Matrix2(m11, m21, m12, m22);
        }

        public static Matrix2 Add(Matrix2 A, float C)
        {
            float m11 = A.M11 + C;
            float m12 = A.M12 + C;
            float m21 = A.M21 + C;
            float m22 = A.M22 + C;

            return new Matrix2(m11, m21, m12, m22);
        }
    }
}
