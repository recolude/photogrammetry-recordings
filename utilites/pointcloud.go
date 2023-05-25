package utilites

import (
	"bytes"
	"fmt"
	"os"

	"github.com/EliCDavis/polyform/formats/ply"
	"github.com/EliCDavis/polyform/modeling"
	"github.com/EliCDavis/vector/vector3"
	rapio "github.com/recolude/rap/format/io"
	"github.com/recolude/rap/format/metadata"
)

func PlyToRapBinary(plyFile string, scale vector3.Float64) (rapio.Binary, error) {
	plyFileHandle, err := os.Open(plyFile)
	if err != nil {
		return rapio.Binary{}, err
	}

	mesh, err := ply.ReadMesh(plyFileHandle)
	if err != nil {
		return rapio.Binary{}, err
	}

	indices := mesh.View().Indices

	var parsedMesh modeling.Mesh
	switch mesh.Topology() {
	case modeling.PointTopology:
		view := mesh.View()
		parsedMesh = modeling.NewPointCloud(map[string][]vector3.Vector[float64]{
			modeling.PositionAttribute: view.Float3Data[modeling.PositionAttribute],
			modeling.ColorAttribute:    view.Float3Data[modeling.ColorAttribute],
		}, nil, nil, nil).
			Scale(vector3.Zero[float64](), scale)

	case modeling.TriangleTopology:
		parsedMesh = mesh.
			CopyFloat3Attribute(*mesh, modeling.PositionAttribute).
			CopyFloat3Attribute(*mesh, modeling.NormalAttribute).
			Scale(vector3.Zero[float64](), scale).
			FlipTriWinding()

	default:
		return rapio.Binary{}, fmt.Errorf("unimplemented topology: %d", mesh.Topology())
	}

	meshData := bytes.Buffer{}
	err = ply.WriteBinary(&meshData, parsedMesh)
	return rapio.NewBinary(plyFile, meshData.Bytes(), metadata.NewBlock(map[string]metadata.Property{
		"points": metadata.NewIntProperty(len(indices)),
	})), err
}
