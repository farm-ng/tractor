package common

import (
	"encoding/base64"
	"encoding/hex"
	"encoding/json"
	"github.com/pion/webrtc/v3"
	"math/rand"
	"time"
)

var LetterRunes = []rune("abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ")

func init() {
	rand.Seed(time.Now().UnixNano())
}

func UniquePeerID() string {
	//return RandStr(7)
	return RandHex(7)
}

// Serialize a SessionDescription object into a base64 byte string
func SerializeSess(sess *webrtc.SessionDescription) (string, error) {
	b, err := json.Marshal(sess)
	if err != nil {
		return "", err
	}
	return base64.StdEncoding.EncodeToString(b), nil
}

// Deserialize base64 byte string into a SessionDescription
func DeserializeSess(sdp string) (*webrtc.SessionDescription, error) {
	sess := &webrtc.SessionDescription{}
	b, err := base64.StdEncoding.DecodeString(sdp)
	if err != nil {
		return nil, err
	}
	err = json.Unmarshal(b, sess)
	if err != nil {
		return nil, err
	}
	return sess, nil
}

func RandStr(n int) string {
	b := make([]rune, n)
	for i := range b {
		b[i] = LetterRunes[rand.Intn(len(LetterRunes))]
	}
	return string(b)
}

func RandHex(n int) string {
	bytes := make([]byte, n)
	if _, err := rand.Read(bytes); err != nil {
		panic(err)
	}
	return hex.EncodeToString(bytes)[:n]
}
