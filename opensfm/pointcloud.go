package opensfm

import (
	"bytes"

	"github.com/EliCDavis/polyform/formats/ply"
	"github.com/EliCDavis/polyform/modeling"
	"github.com/EliCDavis/vector/vector3"
	rapio "github.com/recolude/rap/format/io"
	"github.com/recolude/rap/format/metadata"
)

func PointsToCloudBinary(points map[string]PointSchema) rapio.Binary {
	positionData := make([]vector3.Float64, 0, len(points))
	colorData := make([]vector3.Float64, 0, len(points))

	for _, p := range points {
		positionData = append(positionData, vector3.New(p.Coordinates[0], -p.Coordinates[1], p.Coordinates[2]))
		colorData = append(colorData, vector3.New(p.Color[0], p.Color[1], p.Color[2]).DivByConstant(255.))
	}

	pc := modeling.NewPointCloud(
		map[string][]vector3.Vector[float64]{
			modeling.PositionAttribute: positionData,
			modeling.ColorAttribute:    colorData,
		},
		nil,
		nil,
		nil,
	)

	buf := bytes.Buffer{}
	err := ply.WriteBinary(&buf, pc)
	if err != nil {
		panic(err)
	}

	return rapio.NewBinary("points.ply", buf.Bytes(), metadata.NewBlock(map[string]metadata.Property{
		"points": metadata.NewIntProperty(len(points)),
	}))
}
