package metrics

import (
	"io/ioutil"
	"net/http"

	pb "github.com/farm-ng/tractor/genproto"
	"github.com/farm-ng/tractor/webrtc/internal/eventbus"
	"github.com/gogo/protobuf/proto"
	"github.com/golang/protobuf/ptypes"
	"github.com/golang/snappy"
	"google.golang.org/protobuf/types/known/timestamppb"

	"github.com/prometheus/common/model"
	"github.com/prometheus/prometheus/prompb"
)

// MetricsServer ...
type Server struct {
	EventBus *eventbus.EventBus
}

func (s *Server) ServeHTTP(w http.ResponseWriter, r *http.Request) {
	compressed, err := ioutil.ReadAll(r.Body)
	if err != nil {
		http.Error(w, err.Error(), http.StatusInternalServerError)
		return
	}

	reqBuf, err := snappy.Decode(nil, compressed)
	if err != nil {
		http.Error(w, err.Error(), http.StatusBadRequest)
		return
	}

	var req prompb.WriteRequest
	if err := proto.Unmarshal(reqBuf, &req); err != nil {
		http.Error(w, err.Error(), http.StatusBadRequest)
		return
	}

	for _, ts := range req.Timeseries {

		timeseries := &pb.TimeSeries{}

		event := &pb.Event{}
		event.RecvStamp = ptypes.TimestampNow()
		event.Stamp = &timestamppb.Timestamp{
			Seconds: ts.Samples[0].Timestamp / 1000,
			Nanos:   (int32)(ts.Samples[0].Timestamp%1000) * 1000000,
		}
		event.Name = "metrics"

		labels := &timeseries.Labels
		m := make(model.Metric, len(ts.Labels))
		for _, l := range ts.Labels {
			m[model.LabelName(l.Name)] = model.LabelValue(l.Value)
			*labels = append(*labels, &pb.Label{Name: l.Name, Value: l.Value})
		}
		// fmt.Println(m)

		samples := &timeseries.Samples
		for _, s := range ts.Samples {
			// fmt.Printf("  %f %d\n", s.Value, s.Timestamp)
			*samples = append(*samples, &pb.Sample{Timestamp: s.Timestamp, Value: s.Value})
		}

		event.Data, err = ptypes.MarshalAny(timeseries)
		if err != nil {
			// TODO
		}
		s.EventBus.SendEvent(event)
	}
}
