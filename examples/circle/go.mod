module github.com/nagata-yoshiteru/rvo2-go/examples/circle

replace (
	github.com/nagata-yoshiteru/rvo2-go/monitor => ../../monitor
	github.com/nagata-yoshiteru/rvo2-go/src/rvosimulator => ../../src/rvosimulator
)

require (
	github.com/nagata-yoshiteru/rvo2-go/monitor v0.0.0-00010101000000-000000000000 // indirect
	github.com/nagata-yoshiteru/rvo2-go/src/rvosimulator v0.0.0-00010101000000-000000000000 // indirect
	github.com/mtfelian/golang-socketio v1.5.2 // indirect
)
