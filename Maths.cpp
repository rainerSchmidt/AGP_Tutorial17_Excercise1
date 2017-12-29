#include "maths.h"

ObjFileModel::xyz* Maths::Normalise(ObjFileModel::xyz* v)
{
	ObjFileModel::xyz* vector;
	vector->x = v->x;
	vector->y = v->y;
	vector->z = v->z;

	//Normalise the normal
	float length = sqrt((vector->x * vector->x) + (vector->y * vector->y) + (vector->z * vector->z));
	vector->x /= length;
	vector->y /= length;
	vector->z /= length;

	return vector;
}

float Maths::DotProduct(ObjFileModel::xyz* v1, ObjFileModel::xyz* v2)
{
	float dot = (v1->x * v2->x) + (v1->y * v2->y) + (v1->z * v2->z);
	return dot;
}

ObjFileModel::xyz* Maths::CrossProduct(ObjFileModel::xyz* v1, ObjFileModel::xyz* v2)
{
	ObjFileModel::xyz* Vector3;

	Vector3->x = (v1->y * v2->z) - (v1->z * v2->y);
	Vector3->y = (v1->z * v2->x) - (v1->x * v2->z);
	Vector3->z = (v1->x * v2->y) - (v1->y * v2->x);

	return Vector3;
}

ObjFileModel::xyz* Maths::Normal(ObjFileModel::xyz* A, ObjFileModel::xyz* B, ObjFileModel::xyz* C)
{

	ObjFileModel::xyz* v1;
	ObjFileModel::xyz* v2;

	v1->x = B->x - A->x;
	v1->y = B->y - A->y;
	v1->z = B->z - A->z;

	v2->x= C->x - A->x;
	v2->y = C->y - A->y;
	v2->z = C->z - A->z;

	ObjFileModel::xyz* normal = CrossProduct(v1,v2);

	normal = Normalise(normal);

	return normal;

}

Maths::Plane Maths::PlaneEquation(ObjFileModel::xyz* v1, ObjFileModel::xyz* v2, ObjFileModel::xyz* v3)
{
	//calculate the normal vector to the triangle
	ObjFileModel::xyz* normal = Normal(v1, v2, v3);

	//calculate the d offset
	float d = -DotProduct(v1, normal);

	Maths::Plane plane;
	plane.d = d;
	plane.normal = *normal;

	return plane;
}

float Maths::PlaneEquationForPoint(Plane p, ObjFileModel::xyz* v)
{
	float result = (p.normal.x * v->x) + (p.normal.y * v->y) + (p.normal.z * v->z) + p.d;
	return result;
}

ObjFileModel::xyz Maths::PlaneIntersection(Plane* plane, ObjFileModel::xyz* p1, ObjFileModel::xyz* p2)
{
	ObjFileModel::xyz* intersection;
	intersection->x = 0;
	intersection->y = 0;
	intersection->z = 0;


	// ray
	ObjFileModel::xyz* ray;
	ray->x = p2->x - p1->x;
	ray->y = p2->y - p1->y;
	ray->z = p2->z - p1->z;

	//PlaneEquation for P1 and P2
	int a = CheckSign(PlaneEquationForPoint(*plane, p1));
	int b = CheckSign(PlaneEquationForPoint(*plane, p2));
	
	if (a != b) //if signs are different than intersection
	{
		//calculate IntersectionPoint
		float t = (((-1 * plane->d) - DotProduct(&plane->normal, p1)) / DotProduct(&plane->normal, ray));
		intersection->x = p1->x + (ray->x * t);
		intersection->y = p1->y + (ray->y * t);
		intersection->z = p1->z + (ray->z * t);
		//return IntersectionPoint
		return *intersection;
	}
	//return NullVector;
	return *intersection;


}

bool Maths::InTriangle(ObjFileModel::xyz* A, ObjFileModel::xyz* B, ObjFileModel::xyz* C, ObjFileModel::xyz* P)
{
	ObjFileModel::xyz *vectorAP, *vectorAB, *vectorBP, *vectorBC, *vectorCP, *vectorCA;

	//set vectorAP
	vectorAP->x = P->x - A->x;
	vectorAP->y = P->y - A->y;
	vectorAP->z = P->z - A->z;

	//set vectorAB
	vectorAB->x = B->x - A->x;
	vectorAB->y = B->y - A->y;
	vectorAB->z = B->z - A->z;

	//set vectorBP
	vectorBP->x = P->x - B->x;
	vectorBP->y = P->y - B->y;
	vectorBP->z = P->z - B->z;

	//set vectorBC
	vectorBC->x = C->x - B->x;
	vectorBC->y = C->y - B->y;
	vectorBC->z = C->z - B->z;

	//set vectorCP
	vectorCP->x = P->x - C->x;
	vectorCP->y = P->y - C->y;
	vectorCP->z = P->z - C->z;

	//set vectorCA
	vectorCA->x = A->x - C->x;
	vectorCA->y = A->y - C->y;
	vectorCA->z = A->z - C->z;
	
	float a = CheckSign(DotProduct(CrossProduct(vectorAP, vectorAB), CrossProduct(vectorBP, vectorBC)));
	float b = CheckSign(DotProduct(CrossProduct(vectorBP, vectorBC), CrossProduct(vectorCP, vectorCA)));
	float c = CheckSign(DotProduct(CrossProduct(vectorCP, vectorCA), CrossProduct(vectorAP, vectorAB)));

	//have all DotProduct results the same sign
	if ((a > 0 && b > 0 && c > 0) || a < 0 && b < 0 && c < 0)
		return true;

	return false;
}

int Maths::CheckSign(float number)
{
	return (number < 0.0f ? -1 : (number > 0.0f ? 1 : 0));
}