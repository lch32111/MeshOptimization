#include "matrix.h"

#include "vector.h"

Matrix4 matrix4_translate(Vector3 v)
{
    Matrix4 r;

    r.v[0][0] = 1.f;
    r.v[0][1] = 0.f;
    r.v[0][2] = 0.f;
    r.v[0][3] = 0.f;

    r.v[1][0] = 0.f;
    r.v[1][1] = 1.f;
    r.v[1][2] = 0.f;
    r.v[1][3] = 0.f;

    r.v[2][0] = 0.f;
    r.v[2][1] = 0.f;
    r.v[2][2] = 1.f;
    r.v[2][3] = 0.f;

    r.v[3][0] = v.v[0];
    r.v[3][1] = v.v[1];
    r.v[3][2] = v.v[2];
    r.v[3][3] = 1.f;

    return r;
}

Matrix4 matrix4_scale(Vector3 s)
{
    Matrix4 r;

    r.v[0][0] = s.v[0];
    r.v[0][1] = 0.f;
    r.v[0][2] = 0.f;
    r.v[0][3] = 0.f;

    r.v[1][0] = 0.f;
    r.v[1][1] = s.v[1];
    r.v[1][2] = 0.f;
    r.v[1][3] = 0.f;

    r.v[2][0] = 0.f;
    r.v[2][1] = 0.f;
    r.v[2][2] = s.v[2];
    r.v[2][3] = 0.f;

    r.v[3][0] = 0.f;
    r.v[3][1] = 0.f;
    r.v[3][2] = 0.f;
    r.v[3][3] = 1.f;

    return r;
}

Matrix4 matrix4_mul(const Matrix4* a, const Matrix4* b)
{
    Matrix4 r;

    for (int col = 0; col < 4; ++col)
    {
        for (int row = 0; row < 4; ++row)
        {
            r.v[col][row] = 0.f;

            for (int k = 0; k < 4; ++k)
            {
                r.v[col][row] += a->v[k][row] * b->v[col][k];
            }
        }
    }

    return r;
}

Vector3 matrix4_mul_vec3(const Matrix4* m, Vector3 v)
{
    Vector4 v4 = { v.v[0], v.v[1], v.v[2], 1.f };
    Vector4 r4 = { 0.f, 0.f, 0.f, 0.f };
    for (int col = 0; col < 4; ++col)
    {
        for (int row = 0; row < 4; ++row)
        {
            r4.v[row] += m->v[col][row] * v4.v[col];
        }
    }

    v.v[0] = r4.v[0];
    v.v[1] = r4.v[1];
    v.v[2] = r4.v[2];
    return v;
}

Matrix4 matrix4_mul_scalar(const Matrix4* a, float s)
{
    Matrix4 r;
    for (int col = 0; col < 4; ++col)
    {
        for (int row = 0; row < 4; ++row)
        {
            r.v[col][row] = a->v[col][row] * s;
        }
    }

    return r;
}

// from compute_inverse of glm/glm/detail/func_matrix.inl 
Matrix4 matrix4_inverse(Matrix4* m)
{
    float Coef00 = m->v[2][2] * m->v[3][3] - m->v[3][2] * m->v[2][3];
    float Coef02 = m->v[1][2] * m->v[3][3] - m->v[3][2] * m->v[1][3];
    float Coef03 = m->v[1][2] * m->v[2][3] - m->v[2][2] * m->v[1][3];
    
    float Coef04 = m->v[2][1] * m->v[3][3] - m->v[3][1] * m->v[2][3];
    float Coef06 = m->v[1][1] * m->v[3][3] - m->v[3][1] * m->v[1][3];
    float Coef07 = m->v[1][1] * m->v[2][3] - m->v[2][1] * m->v[1][3];
    
    float Coef08 = m->v[2][1] * m->v[3][2] - m->v[3][1] * m->v[2][2];
    float Coef10 = m->v[1][1] * m->v[3][2] - m->v[3][1] * m->v[1][2];
    float Coef11 = m->v[1][1] * m->v[2][2] - m->v[2][1] * m->v[1][2];
    
    float Coef12 = m->v[2][0] * m->v[3][3] - m->v[3][0] * m->v[2][3];
    float Coef14 = m->v[1][0] * m->v[3][3] - m->v[3][0] * m->v[1][3];
    float Coef15 = m->v[1][0] * m->v[2][3] - m->v[2][0] * m->v[1][3];
    
    float Coef16 = m->v[2][0] * m->v[3][2] - m->v[3][0] * m->v[2][2];
    float Coef18 = m->v[1][0] * m->v[3][2] - m->v[3][0] * m->v[1][2];
    float Coef19 = m->v[1][0] * m->v[2][2] - m->v[2][0] * m->v[1][2];
    
    float Coef20 = m->v[2][0] * m->v[3][1] - m->v[3][0] * m->v[2][1];
    float Coef22 = m->v[1][0] * m->v[3][1] - m->v[3][0] * m->v[1][1];
    float Coef23 = m->v[1][0] * m->v[2][1] - m->v[2][0] * m->v[1][1];

    Vector4 Fac0 = vector4_set4( Coef00, Coef00, Coef02, Coef03 );
    Vector4 Fac1 = vector4_set4( Coef04, Coef04, Coef06, Coef07 );
    Vector4 Fac2 = vector4_set4( Coef08, Coef08, Coef10, Coef11 );
    Vector4 Fac3 = vector4_set4( Coef12, Coef12, Coef14, Coef15 );
    Vector4 Fac4 = vector4_set4( Coef16, Coef16, Coef18, Coef19 );
    Vector4 Fac5 = vector4_set4( Coef20, Coef20, Coef22, Coef23 );

    Vector4 Vec0 = vector4_set4( m->v[1][0], m->v[0][0], m->v[0][0], m->v[0][0] );
    Vector4 Vec1 = vector4_set4( m->v[1][1], m->v[0][1], m->v[0][1], m->v[0][1] );
    Vector4 Vec2 = vector4_set4( m->v[1][2], m->v[0][2], m->v[0][2], m->v[0][2] );
    Vector4 Vec3 = vector4_set4( m->v[1][3], m->v[0][3], m->v[0][3], m->v[0][3] );

    Vector4 Inv0 = vector4_add(vector4_sub(vector4_mul(Vec1, Fac0), vector4_mul(Vec2, Fac1)), vector4_mul(Vec3, Fac2));
    Vector4 Inv1 = vector4_add(vector4_sub(vector4_mul(Vec0, Fac0), vector4_mul(Vec2, Fac3)), vector4_mul(Vec3, Fac4));
    Vector4 Inv2 = vector4_add(vector4_sub(vector4_mul(Vec0, Fac1), vector4_mul(Vec1, Fac3)), vector4_mul(Vec3, Fac5));
    Vector4 Inv3 = vector4_add(vector4_sub(vector4_mul(Vec0, Fac2), vector4_mul(Vec1, Fac4)), vector4_mul(Vec2, Fac5));

    Vector4 SignA = vector4_set4(+1, -1, +1, -1);
    Vector4 SignB = vector4_set4(-1, +1, -1, +1);

    Matrix4 r;

    Vector4 t0 = vector4_mul(Inv0, SignA);
    Vector4 t1 = vector4_mul(Inv1, SignB);
    Vector4 t2 = vector4_mul(Inv2, SignA);
    Vector4 t3 = vector4_mul(Inv3, SignB);
    for (int i = 0; i < 4; ++i)
    {
        r.v[0][i] = t0.v[i];
        r.v[1][i] = t1.v[i];
        r.v[2][i] = t2.v[i];
        r.v[3][i] = t3.v[i];
    }
        
    Vector4 Row0 = vector4_set4(r.v[0][0], r.v[1][0], r.v[2][0], r.v[3][0]);

    Vector4 Dot0 = vector4_mul(Row0, vector4_set4(m->v[0][0], m->v[0][1], m->v[0][2], m->v[0][3]));
    float Dot1 = (Dot0.v[0] + Dot0.v[1]) + (Dot0.v[2] + Dot0.v[3]);

    float OneOverDeterminant = 1.f / Dot1;

    float* v = (float*)r.v;
    for (int i = 0; i < 16; ++i)
    {
        v[i] = v[i] * OneOverDeterminant;
    }

    return r;
}