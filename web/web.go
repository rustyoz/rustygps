package web

import (
	"encoding/json"
	"log"
	"net/http"

	"github.com/rustyoz/rustygps/gps"
)

func Run(port string) {

	// serve the tractor location at "/gps/position"
	http.HandleFunc("/gps/position", func(w http.ResponseWriter, r *http.Request) {

		// get the gps position
		position := gps.GetPosition()

		// serve the position as json
		w.Header().Set("Content-Type", "application/json")
		json.NewEncoder(w).Encode(position)
	})

	// serve the static files
	http.Handle("/web/static/", http.StripPrefix("/web/static/", http.FileServer(http.Dir("web/static"))))

	// serve the web interface
	err := http.ListenAndServe(port, nil)
	if err != nil {
		log.Println("Error starting web server: ", err)
	} else {
		log.Println("Web server started on port ", port)
	}

}
