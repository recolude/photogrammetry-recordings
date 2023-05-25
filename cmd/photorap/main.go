package main

import (
	"encoding/json"
	"fmt"
	"log"
	"math"
	"os"
	"regexp"
	"sort"
	"strconv"

	"github.com/EliCDavis/vector/vector3"
	"github.com/recolude/photogrammetry-recordings/opensfm"
	"github.com/recolude/photogrammetry-recordings/utilites"
	"github.com/recolude/rap/format"
	"github.com/recolude/rap/format/collection/euler"
	"github.com/recolude/rap/format/collection/event"
	"github.com/recolude/rap/format/collection/position"
	"github.com/recolude/rap/format/encoding"
	eulEnc "github.com/recolude/rap/format/encoding/euler"
	eventEnc "github.com/recolude/rap/format/encoding/event"
	posEnc "github.com/recolude/rap/format/encoding/position"
	rapio "github.com/recolude/rap/format/io"
	"github.com/recolude/rap/format/metadata"
	"github.com/urfave/cli/v2"
)

func shotsHaveTimestamp(shots map[string]opensfm.ShotSchema) bool {
	for _, shot := range shots {
		if shot.CaptureTime != 0 {
			return true
		}
	}
	return false
}

type SortPositionByTime []position.Capture

func (a SortPositionByTime) Len() int           { return len(a) }
func (a SortPositionByTime) Swap(i, j int)      { a[i], a[j] = a[j], a[i] }
func (a SortPositionByTime) Less(i, j int) bool { return a[i].Time() < a[j].Time() }

type SortRotationByTime []euler.Capture

func (a SortRotationByTime) Len() int           { return len(a) }
func (a SortRotationByTime) Swap(i, j int)      { a[i], a[j] = a[j], a[i] }
func (a SortRotationByTime) Less(i, j int) bool { return a[i].Time() < a[j].Time() }

type SortEventByTime []event.Capture

func (a SortEventByTime) Len() int           { return len(a) }
func (a SortEventByTime) Swap(i, j int)      { a[i], a[j] = a[j], a[i] }
func (a SortEventByTime) Less(i, j int) bool { return a[i].Time() < a[j].Time() }

func openSFMCameraToSubject(recon opensfm.ReconstructionSchema, cameraID string) format.Recording {
	re := regexp.MustCompile("[0-9]+")

	inferTimestamps := !shotsHaveTimestamp(recon.Shots)

	positionCaptures := make([]position.Capture, 0, len(recon.Shots))
	rotationCaptures := make([]euler.Capture, 0, len(recon.Shots))
	eventCaptures := make([]event.Capture, 0, len(recon.Shots))

	for name, shot := range recon.Shots {
		if shot.Camera != cameraID {
			continue
		}
		shotIndexStr := re.FindString(name)
		shotIndex, err := strconv.Atoi(shotIndexStr)
		if err != nil {
			panic(err)
		}

		time := shot.CaptureTime
		if inferTimestamps {
			time = float64(shotIndex)
		}
		positionCaptures = append(positionCaptures, position.NewCapture(time, shot.Translation[0], -shot.Translation[1], shot.Translation[2]))
		rotationCaptures = append(rotationCaptures, euler.NewEulerZXYCapture(
			time,
			float64(int(math.Round(shot.Rotation[0]*180.))%360),
			float64(int(math.Round(shot.Rotation[1]*180.))%360),
			float64(int(math.Round(shot.Rotation[2]*180.))%360),
		))
		eventCaptures = append(eventCaptures, event.NewCapture(time, name, metadata.NewBlock(map[string]metadata.Property{
			"Orientation Index": metadata.NewIntProperty(shot.Orientation),
			"Scale":             metadata.NewFloat32Property(float32(shot.Scale)),
		})))
	}

	sort.Sort(SortPositionByTime(positionCaptures))
	sort.Sort(SortRotationByTime(rotationCaptures))
	sort.Sort(SortEventByTime(eventCaptures))

	camera := recon.Cameras[cameraID]

	return format.NewRecording(
		cameraID,
		cameraID,
		[]format.CaptureCollection{
			position.NewCollection("Position", positionCaptures),
			euler.NewCollection("Rotation", rotationCaptures),
			event.NewCollection("Custom Event", eventCaptures),
		},
		nil,
		metadata.NewBlock(map[string]metadata.Property{
			"Projection Type": metadata.NewStringProperty(camera.ProjectionType),
			"Width":           metadata.NewIntProperty(camera.Width),
			"Height":          metadata.NewIntProperty(camera.Height),
			"Focal":           metadata.NewFloat32Property(float32(camera.Focal)),
			"K1":              metadata.NewFloat32Property(float32(camera.K1)),
			"K2":              metadata.NewFloat32Property(float32(camera.K2)),
		}),
		[]format.Binary{},
		[]format.BinaryReference{},
	)
}

func openSFMShotsToSubjects(recon opensfm.ReconstructionSchema) []format.Recording {
	recordings := make([]format.Recording, 0, len(recon.Cameras))
	for id := range recon.Cameras {
		recordings = append(recordings, openSFMCameraToSubject(recon, id))
	}
	return recordings
}

func openSfmReconstructionToRecording(recon opensfm.ReconstructionSchema, bins []format.Binary) format.Recording {
	allBins := bins
	allBins = append(allBins, opensfm.PointsToCloudBinary(recon.Points))
	return format.NewRecording(
		"opensfm",
		"Open SFM",
		[]format.CaptureCollection{},
		openSFMShotsToSubjects(recon),
		metadata.NewBlock(map[string]metadata.Property{
			"recolude-grid": metadata.NewBoolProperty(true),
			"cameras":       metadata.NewIntProperty(len(recon.Cameras)),
			"shots":         metadata.NewIntProperty(len(recon.Shots)),
			"points":        metadata.NewIntProperty(len(recon.Points)),
		}),
		allBins,
		[]format.BinaryReference{},
	)
}

func main() {
	app := &cli.App{
		Name:  "Photogrammetry Utils",
		Usage: "Converts OpenSFM reconstruction data to RAP",
		Authors: []*cli.Author{
			{
				Name:  "Eli Davis",
				Email: "eli@recolude.com",
			},
		},
		Commands: []*cli.Command{
			{
				Name: "opensfm",
				Flags: []cli.Flag{
					&cli.StringFlag{
						Name:  "reconstruction",
						Usage: "path to openSFM reconstruction file",
					},
					&cli.StringSliceFlag{
						Name:  "openmvs",
						Usage: "Path to openMVS data to include",
					},
					&cli.StringFlag{
						Name:  "out",
						Usage: "path to rap file",
					},
				},
				Action: func(c *cli.Context) error {
					reconstructionData, err := os.ReadFile(c.String("reconstruction"))
					if err != nil {
						return err
					}

					var reconstructionFile opensfm.ReconstructionJsonSchema
					if err := json.Unmarshal(reconstructionData, &reconstructionFile); err != nil {
						return err
					}

					if len(reconstructionFile) > 1 {
						return fmt.Errorf("unimplemented scenario where reconstruction json contained more than one reconstruction")
					}

					f, err := os.Create(c.String("out"))
					if err != nil {
						return err
					}
					defer f.Close()

					rapWriter := rapio.NewWriter(
						[]encoding.Encoder{
							posEnc.NewEncoder(posEnc.Oct24),
							eulEnc.NewEncoder(eulEnc.Raw16),
							eventEnc.NewEncoder(),
						},
						true,
						f,
						rapio.BST16,
					)

					extraBins := make([]format.Binary, 0)
					if c.IsSet("openmvs") {
						for _, v := range c.StringSlice("openmvs") {
							bin, err := utilites.PlyToRapBinary(v, vector3.New[float64](1, -1, 1))
							if err != nil {
								return err
							}
							extraBins = append(extraBins, bin)
						}
					}

					_, err = rapWriter.Write(openSfmReconstructionToRecording(reconstructionFile[0], extraBins))

					return err
				},
			},
		},
	}

	if err := app.Run(os.Args); err != nil {
		log.Fatal(err)
	}
}
