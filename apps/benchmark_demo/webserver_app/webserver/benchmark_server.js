#!/usr/bin/env node

/*
 * Libraries 
 */

var serveStatic = require('serve-static')
var express = require('express')
var editJsonFile = require('edit-json-file')

/*
 * Definitions
 */

/* Set the path */
var demo_dir = process.argv[2]

/* Create the express app */
var app = express();

/* Point to JSON data for writing */
var json_file = editJsonFile(demo_dir + 'oob_data.json', {
	autosave: true
});

/*
 * Server Code
 */

/* Place the GUI files onto the server and point to the main html file */
app.use(serveStatic(demo_dir, {'index': ['index.html']}))

/* Handle incoming POST requests */
app.post("/oob_data.json", function(request, response) {
	let body = '';
	request.on('data', chunk => {
		body += chunk.toString();
	});
	request.on('end', () => {
		try {
			/* Parse the json data into an array using a regex, and then filter out the empty array entries */
			fixed_data = body.split(/{?"?([A-Za-z0-9_]+)"?:?}*/).filter(n => n);

			/* Update the json file with the newly received data */
			json_file.set(fixed_data[0] + '.' + fixed_data[1] + '.' + fixed_data[2], Number(fixed_data[3]));
			json_file.set(fixed_data[4] + '.' + fixed_data[5] + '.' + fixed_data[6], Number(fixed_data[7]));
			json_file.set(fixed_data[0] + '.' + fixed_data[1] + '.changed', 1);
			
			/* End the POST response */
			response.end(body);
		} catch (error) {
			/* End the POST response with an error status code */
			response.statusCode = 400;
			response.end(error.toString());
		}
	});
});

/* Open the server on port 8081 */
app.listen(8081);
