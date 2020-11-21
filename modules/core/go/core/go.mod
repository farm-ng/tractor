module github.com/farm-ng/core

go 1.14

require (
	github.com/farm-ng/genproto/core v0.0.0-00010101000000-000000000000
	github.com/golang/protobuf v1.4.2
	golang.org/x/net v0.0.0-20200625001655-4c5254603344
	golang.org/x/sys v0.0.0-20200831180312-196b9ba8737a
	google.golang.org/protobuf v1.25.0
)

replace (
	github.com/farm-ng/genproto/core => ../../../../build/modules/core/protos/go/github.com/farm-ng/genproto/core
)
