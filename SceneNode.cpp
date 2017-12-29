#include "scenenode.h"

SceneNode::SceneNode(float X, float Y, float Z, float RotX, float RotY, float RotZ, float Scale)
{
	m_pModel = NULL;

	m_x = X;
	m_y = Y;
	m_z = Z;
	m_xangle = RotX;
	m_yangle = RotY;
	m_zangle = RotZ;
	m_scale = Scale;
}

void SceneNode::AddChildNode(SceneNode* Node)
{
	m_children.push_back(Node);
}

bool SceneNode::DetatchNode(SceneNode* Node)
{
	//traverse tree to find node to detatch
	for (int i = 0; i < m_children.size(); i++)
	{
		if (Node == m_children[i])
		{
			m_children.erase(m_children.begin() + i);
			return true;
		}
		if (m_children[i]->DetatchNode(Node) == true) return true;
	}

	return false; //note not in this tree
}

void SceneNode::Execute(XMMATRIX* World, XMMATRIX* View, XMMATRIX* Projection)
{
	//the local_world_matrix will be used to calc the local transformations fir this node
	XMMATRIX local_world = XMMatrixIdentity();

	local_world = XMMatrixScaling(m_scale, m_scale, m_scale);
	local_world *= XMMatrixRotationX(XMConvertToRadians(m_xangle));
	local_world *= XMMatrixRotationY(XMConvertToRadians(m_yangle));
	local_world *= XMMatrixRotationZ(XMConvertToRadians(m_zangle));
	local_world *= XMMatrixTranslation(m_x, m_y, m_z);

	//the local matrix is multiplied by the passed in world matrix that contains the concatenated
	//transformations of all parent nodes so that this nodes transformations are relative to those
	local_world *= *World;

	//only draw if there is a model attached
	if (m_pModel) m_pModel->Draw(&local_world, View, Projection);

	//traverse all child nodes, passing in the concatenated world matrix
	for (int i = 0; i < m_children.size(); i++)
	{
		m_children[i]->Execute(&local_world, View, Projection);
	}	
}

void SceneNode::LookAt_XZ(float X, float Z)
{
	float DX = X - m_x;
	float DZ = Z - m_z;

	m_yangle = atan2(DX, DZ) * (180.0 / XM_PI);
}

bool SceneNode::MoveForwards(float Distance, SceneNode* RootNode)
{
	float old_x = m_x;
	float old_z = m_z;

	m_x += sin(m_yangle * (XM_PI / 180.0)) * Distance;
	m_z += cos(m_yangle * (XM_PI / 180.0)) * Distance;

	XMMATRIX Identity = XMMatrixIdentity();

	//since state has changed, need to update collision tree
	//this vasic system requires entire hierarchy to be updated
	//so start at root_node passing in identity matrix
	RootNode->UpdateCollisionTree(&Identity, 1.0);
	if (CheckCollision(RootNode) == true)
	{
		// if collision restore state
		m_x = old_x;
		m_z = old_z;

		return true;
	}

	return false;
}

bool SceneNode::CheckCollision(SceneNode* CompareTree)
{
	return CheckCollision(CompareTree, this);
}

bool SceneNode::CheckCollision(SceneNode* CompareTree, SceneNode* ObjectTreeRoot)
{
	// check to see if root of tree being compared is same as root node of object tree being checked
	// i.e. stop pbject node and children being checked against eachother
	if (ObjectTreeRoot == CompareTree) return false;

	// only check for collisions if both nodes contain a model
	if (m_pModel && CompareTree->m_pModel)
	{
		XMVECTOR v1 = GetWorldCenterPosition();
		XMVECTOR v2 = CompareTree->GetWorldCenterPosition();
		XMVECTOR vdiff = v1 - v2;

		float x1 = XMVectorGetX(v1);
		float y1 = XMVectorGetY(v1);
		float z1 = XMVectorGetZ(v1);
		float x2 = XMVectorGetX(v2);
		float y2 = XMVectorGetY(v2);
		float z2 = XMVectorGetZ(v2);

		float dx = x1 - x2;
		float dy = y1 - y2;
		float dz = z1 - z2;

		// check bounding sphere collision
		if (sqrt(dx*dx + dy*dy + dz*dz) <
			(CompareTree->m_pModel->GetBoundingSphereRadius() * CompareTree->m_world_scale) +
			(this->m_pModel->GetBoundingSphereRadius() * m_world_scale))
		{
			return true;
		}
	}

	// iterate through compared tree child nodes
	for (int i = 0; i < CompareTree->m_children.size(); i++)
	{
		//check for collision against all compared tree child nodes
		if (CheckCollision(CompareTree->m_children[i], ObjectTreeRoot) == true) return true;
	}

	// iterate through composite object child nodes
	for (int i = 0; i < m_children.size(); i++)
	{
		// check all the child nodes of the composite object against compared tree
		if (m_children[i]->CheckCollision(CompareTree, ObjectTreeRoot) == true) return true;
	}

	return false;
}

void SceneNode::UpdateCollisionTree(XMMATRIX* World, float Scale)
{
	//XMMATRIX local_world = XMMatrixIdentity();
	m_local_world_matrix = XMMatrixIdentity();

	m_local_world_matrix = XMMatrixScaling(m_scale, m_scale, m_scale);
	m_local_world_matrix *= XMMatrixRotationX(XMConvertToRadians(m_xangle));
	m_local_world_matrix *= XMMatrixRotationY(XMConvertToRadians(m_yangle));
	m_local_world_matrix *= XMMatrixRotationZ(XMConvertToRadians(m_zangle));
	m_local_world_matrix *= XMMatrixTranslation(m_x, m_y, m_z);

	m_local_world_matrix *= *World;
	m_world_scale = Scale * m_scale;

	XMVECTOR v;
	if (m_pModel)
	{
		v = XMVectorSet(m_pModel->GetBoundingSphereCentreX(),
						m_pModel->GetBoundingSphereCentreY(),
						m_pModel->GetBoundingSphereCentreZ(), 0.0);
	}
	else
	{
		v = XMVectorSet(0, 0, 0, 0); // no Model, default to 0
	}

	// find and store world space bounding sphere center
	v = XMVector3Transform(v, m_local_world_matrix);
	m_world_centerX = XMVectorGetX(v);
	m_world_centerY = XMVectorGetY(v);
	m_world_centerZ = XMVectorGetZ(v);

	// traverse all childe nodes, passing in the concatenated world matrix and scale
	for (int i = 0; i < m_children.size(); i++)
	{
		m_children[i]->UpdateCollisionTree(&m_local_world_matrix, m_world_scale);
	}

}

//SETTER
void SceneNode::SetPosX(float X) { m_x = X; }
void SceneNode::SetPosY(float Y) { m_y = Y; }
void SceneNode::SetPosZ(float Z) { m_z = Z; }
void SceneNode::SetRotX(float Amount) { m_xangle = Amount; }
void SceneNode::AddRotX(float Amount) { m_xangle += Amount; }
void SceneNode::SetRotY(float Amount) { m_yangle = Amount; }
void SceneNode::SetRotZ(float Amount) { m_zangle = Amount; }
//void SceneNode::SetScale(float Amount) { m_scale = Amount; }
void SceneNode::SetModel(Model* Model) { m_pModel = Model; }

//GETTER
Model* SceneNode::GetModel() { return m_pModel; };
XMVECTOR SceneNode::GetWorldCenterPosition()
{
	return XMVectorSet(m_world_centerX, m_world_centerY, m_world_centerZ,0.0);
}