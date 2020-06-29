module github.com/nagata-yoshiteru/rvo2-go/examples/blocks

require (
	github.com/nagata-yoshiteru/rvo2-go v1.1.0 // indirect
	github.com/nagata-yoshiteru/rvo2-go/monitor v0.0.0-20200115064247-262dcb97d11d // indirect
	github.com/nagata-yoshiteru/rvo2-go/src/rvosimulator v0.0.0-20200115064247-262dcb97d11d // indirect
	github.com/mtfelian/golang-socketio v1.5.2 // indirect
)

replace (
	github.com/nagata-yoshiteru/rvo2-go/monitor => ../../monitor
	github.com/nagata-yoshiteru/rvo2-go/src/rvosimulator => ../../src/rvosimulator
)
