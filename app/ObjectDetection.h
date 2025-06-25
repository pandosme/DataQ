/*------------------------------------------------------------------
 *  Fred Juhlin (2025)
 
/*
 Input data from Message Broker
 {
	"frame":{
		"observations":[
			{
				"bounding_box":{
					"bottom":0.1927,
					"left":0.4838,
					"right":0.5138,
					"top":0.155
				},
				"class":{
					"colors":[
						{
							"name":"Gray",
							"score":0.57
						}
					],
					"score":0.81,
					"type":"Car", ["Human","Bike","Bus",Truck","Face","Vehicle"]
					"lower_clothing_colors":[
						{
							"name":"Black", ["Black","Red",White","Blue",Yellow",Gray"]
							"score":0.36
						}
					],
					"upper_clothing_colors":[
						{
							"name":"Black",
							"score":0.33
						}
					]
				},
				"image": {
					"bounding_box":{
						"bottom":0.196,
						"left":0.4804,
						"right":0.5172,
						"top":0.1516
					},
					"data":"",
					"timestamp":"2025-06-17T21:13:31.927203Z"
				},
				"timestamp":"2025-06-17T21:13:31.927203Z",
				"track_id":"de1970dc-f7d1-40e6-8309-61825539d923"
			}
		],
		"operations": [
			{
				"id": "195a6d06-048c-49dd-a71b-1379ca90faa0",
				"type": "DeleteOperation"
			}
		],
		"timestamp":"2025-06-17T21:13:31.927203Z"
	}
	
	Output data to user callback
	[
		{
			id: "de1970dc-f7d1-40e6-8309-61825539d923",
			active: true,   	  //When false, the object is lost or it left the scene
			class: "Car",		  //"Black","Red",White","Blue",Yellow",Gray"
			confidence: [0-100],  //The highest registered confidence level
			x,y,w,h: [0-1000],    //The current position
			age: float,			  //How many seconds in scene
			idle: float,		  //The number of seconds the object has been stationary,
			cx,cy: [0-1000],      //The current center of gravirty.  Either middle of the object or center-bottom depending on configuration
			bx,by: [0-1000],      //The birth cx, cy position the object was first detected
			dx,dy: [0-1000],      //The totdal deleta displacement from birth position. Negative means left/up and positive means right/down
			distance: int,		  //Total percent of scene area movement
			color: "Black",	      //Vehicle color or human upper body
			color1: "Red"		  //Human lower body or second vehicle color (if applicable)
			timestamp: [EPOCH ms] //Current timestamp
		},
		{...}
	]
}
 
 *------------------------------------------------------------------*/
 
#ifndef ObjectDetection_H
#define ObjectDetection_H

#include "cJSON.h"

typedef void (*ObjectDetection_Callback)( cJSON *detections  );
//Consumer needs delete list;

int		ObjectDetection_Init( ObjectDetection_Callback callback );
void	ObjectDetection_Config( cJSON* data );
void	ObjectDetection_Reset();
int		ObjectDetection_CacheSize();

#endif
