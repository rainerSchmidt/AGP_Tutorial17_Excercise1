struct VOut
{
	float4 position : SV_POSITION;
	float4 color : COLOR;
	float2 texcoord : TEXCOORD;
	//float3 normal : NORMAL;
};

Texture2D		texture0;
SamplerState	sampler0;

cbuffer CBuffer0
{
	matrix WVPMatrix; // 64 bytes
	//float threeColor_fraction; // 4 bytes
	//float scale;    // 4 bytes
	//float2 packing; // 2x4 bytes = 8 bytes
	float4 directional_light_vector; // 16 Bytes
	float4 directional_light_colour; // 16 Bytes
	float4 ambient_light_colour; // 16 Bytes
	// total 112 bytes
};

VOut VShader(float4 position : POSITION, float4 color : COLOR, float2 texcoord : TEXCOORD, float3 normal : NORMAL)
{
	VOut output;

	//color.g *= threeColor_fraction;
	output.position = mul(WVPMatrix, position);
	//output.position = position;
	

	float diffuse_amount = dot(directional_light_vector, normal);

	diffuse_amount = saturate(diffuse_amount);

	output.color = ambient_light_colour + (directional_light_colour * diffuse_amount);

	output.texcoord = texcoord;

	return output;
}

float4 PShader(float4 position : SV_POSITION, float4 color : COLOR, float2 texcoord : TEXCOORD) : SV_TARGET
{
	return color * texture0.Sample(sampler0, texcoord);
}