;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;
;;; $Id$
;;;
;;; Copyright (c) 1987- JSK, The University of Tokyo.  All Rights Reserved.
;;;
;;; This software is a collection of EusLisp code for robot applications,
;;; which has been developed by the JSK Laboratory for the IRT project.
;;; For more information on EusLisp and its application to the robotics,
;;; please refer to the following papers.
;;;
;;; Toshihiro Matsui
;;; Multithread object-oriented language euslisp for parallel and
;;;  asynchronous programming in robotics
;;; Workshop on Concurrent Object-based Systems,
;;;  IEEE 6th Symposium on Parallel and Distributed Processing, 1994
;;;
;;; Permission to use this software for educational, research
;;; and non-profit purposes, without fee, and without a written
;;; agreement is hereby granted to all researchers working on
;;; the IRT project at the University of Tokyo, provided that the
;;; above copyright notice remains intact.  
;;;

(in-package "GL")

(defvar *face-colors*   ;; created X11R6/lib/X11/rgb.txt
  (list
   (instance colormaterial
		  :create :name :snow
		  :diffuse #f(0.99609375 0.9765625 0.9765625))
	(instance colormaterial
		  :create :name :GhostWhite
		  :diffuse #f(0.96875 0.96875 0.99609375))
	(instance colormaterial
		  :create :name :WhiteSmoke
		  :diffuse #f(0.95703125 0.95703125 0.95703125))
	(instance colormaterial
		  :create :name :gainsboro
		  :diffuse #f(0.859375 0.859375 0.859375))
	(instance colormaterial
		  :create :name :FloralWhite
		  :diffuse #f(0.99609375 0.9765625 0.9375))
	(instance colormaterial
		  :create :name :OldLace
		  :diffuse #f(0.98828125 0.95703125 0.8984375))
	(instance colormaterial
		  :create :name :linen
		  :diffuse #f(0.9765625 0.9375 0.8984375))
	(instance colormaterial
		  :create :name :AntiqueWhite
		  :diffuse #f(0.9765625 0.91796875 0.83984375))
	(instance colormaterial
		  :create :name :PapayaWhip
		  :diffuse #f(0.99609375 0.93359375 0.83203125))
	(instance colormaterial
		  :create :name :BlanchedAlmond
		  :diffuse #f(0.99609375 0.91796875 0.80078125))
	(instance colormaterial
		  :create :name :bisque
		  :diffuse #f(0.99609375 0.890625 0.765625))
	(instance colormaterial
		  :create :name :PeachPuff
		  :diffuse #f(0.99609375 0.8515625 0.72265625))
	(instance colormaterial
		  :create :name :NavajoWhite
		  :diffuse #f(0.99609375 0.8671875 0.67578125))
	(instance colormaterial
		  :create :name :moccasin
		  :diffuse #f(0.99609375 0.890625 0.70703125))
	(instance colormaterial
		  :create :name :cornsilk
		  :diffuse #f(0.99609375 0.96875 0.859375))
	(instance colormaterial
		  :create :name :ivory
		  :diffuse #f(0.99609375 0.99609375 0.9375))
	(instance colormaterial
		  :create :name :LemonChiffon
		  :diffuse #f(0.99609375 0.9765625 0.80078125))
	(instance colormaterial
		  :create :name :seashell
		  :diffuse #f(0.99609375 0.95703125 0.9296875))
	(instance colormaterial
		  :create :name :honeydew
		  :diffuse #f(0.9375 0.99609375 0.9375))
	(instance colormaterial
		  :create :name :MintCream
		  :diffuse #f(0.95703125 0.99609375 0.9765625))
	(instance colormaterial
		  :create :name :azure
		  :diffuse #f(0.9375 0.99609375 0.99609375))
	(instance colormaterial
		  :create :name :AliceBlue
		  :diffuse #f(0.9375 0.96875 0.99609375))
	(instance colormaterial
		  :create :name :lavender
		  :diffuse #f(0.8984375 0.8984375 0.9765625))
	(instance colormaterial
		  :create :name :LavenderBlush
		  :diffuse #f(0.99609375 0.9375 0.95703125))
	(instance colormaterial
		  :create :name :MistyRose
		  :diffuse #f(0.99609375 0.890625 0.87890625))
	(instance colormaterial
		  :create :name :white
		  :diffuse #f(0.99609375 0.99609375 0.99609375))
	(instance colormaterial
		  :create :name :black
		  :diffuse #f(0 0 0))
	(instance colormaterial
		  :create :name :DarkSlateGray
		  :diffuse #f(0.18359375 0.30859375 0.30859375))
	(instance colormaterial
		  :create :name :DarkSlateGrey
		  :diffuse #f(0.18359375 0.30859375 0.30859375))
	(instance colormaterial
		  :create :name :DimGray
		  :diffuse #f(0.41015625 0.41015625 0.41015625))
	(instance colormaterial
		  :create :name :DimGrey
		  :diffuse #f(0.41015625 0.41015625 0.41015625))
	(instance colormaterial
		  :create :name :SlateGray
		  :diffuse #f(0.4375 0.5 0.5625))
	(instance colormaterial
		  :create :name :SlateGrey
		  :diffuse #f(0.4375 0.5 0.5625))
	(instance colormaterial
		  :create :name :LightSlateGray
		  :diffuse #f(0.46484375 0.53125 0.59765625))
	(instance colormaterial
		  :create :name :LightSlateGrey
		  :diffuse #f(0.46484375 0.53125 0.59765625))
	(instance colormaterial
		  :create :name :gray
		  :diffuse #f(0.7421875 0.7421875 0.7421875))
	(instance colormaterial
		  :create :name :grey
		  :diffuse #f(0.7421875 0.7421875 0.7421875))
	(instance colormaterial
		  :create :name :LightGrey
		  :diffuse #f(0.82421875 0.82421875 0.82421875))
	(instance colormaterial
		  :create :name :LightGray
		  :diffuse #f(0.82421875 0.82421875 0.82421875))
	(instance colormaterial
		  :create :name :MidnightBlue
		  :diffuse #f(0.09765625 0.09765625 0.4375))
	(instance colormaterial
		  :create :name :navy
		  :diffuse #f(0 0 0.5))
	(instance colormaterial
		  :create :name :NavyBlue
		  :diffuse #f(0 0 0.5))
	(instance colormaterial
		  :create :name :CornflowerBlue
		  :diffuse #f(0.390625 0.58203125 0.92578125))
	(instance colormaterial
		  :create :name :DarkSlateBlue
		  :diffuse #f(0.28125 0.23828125 0.54296875))
	(instance colormaterial
		  :create :name :SlateBlue
		  :diffuse #f(0.4140625 0.3515625 0.80078125))
	(instance colormaterial
		  :create :name :MediumSlateBlue
		  :diffuse #f(0.48046875 0.40625 0.9296875))
	(instance colormaterial
		  :create :name :LightSlateBlue
		  :diffuse #f(0.515625 0.4375 0.99609375))
	(instance colormaterial
		  :create :name :MediumBlue
		  :diffuse #f(0 0 0.80078125))
	(instance colormaterial
		  :create :name :RoyalBlue
		  :diffuse #f(0.25390625 0.41015625 0.87890625))
	(instance colormaterial
		  :create :name :blue
		  :diffuse #f(0 0 0.99609375))
	(instance colormaterial
		  :create :name :DodgerBlue
		  :diffuse #f(0.1171875 0.5625 0.99609375))
	(instance colormaterial
		  :create :name :DeepSkyBlue
		  :diffuse #f(0 0.74609375 0.99609375))
	(instance colormaterial
		  :create :name :SkyBlue
		  :diffuse #f(0.52734375 0.8046875 0.91796875))
	(instance colormaterial
		  :create :name :LightSkyBlue
		  :diffuse #f(0.52734375 0.8046875 0.9765625))
	(instance colormaterial
		  :create :name :SteelBlue
		  :diffuse #f(0.2734375 0.5078125 0.703125))
	(instance colormaterial
		  :create :name :LightSteelBlue
		  :diffuse #f(0.6875 0.765625 0.8671875))
	(instance colormaterial
		  :create :name :LightBlue
		  :diffuse #f(0.67578125 0.84375 0.8984375))
	(instance colormaterial
		  :create :name :PowderBlue
		  :diffuse #f(0.6875 0.875 0.8984375))
	(instance colormaterial
		  :create :name :PaleTurquoise
		  :diffuse #f(0.68359375 0.9296875 0.9296875))
	(instance colormaterial
		  :create :name :DarkTurquoise
		  :diffuse #f(0 0.8046875 0.81640625))
	(instance colormaterial
		  :create :name :MediumTurquoise
		  :diffuse #f(0.28125 0.81640625 0.796875))
	(instance colormaterial
		  :create :name :turquoise
		  :diffuse #f(0.25 0.875 0.8125))
	(instance colormaterial
		  :create :name :cyan
		  :diffuse #f(0 0.99609375 0.99609375))
	(instance colormaterial
		  :create :name :LightCyan
		  :diffuse #f(0.875 0.99609375 0.99609375))
	(instance colormaterial
		  :create :name :CadetBlue
		  :diffuse #f(0.37109375 0.6171875 0.625))
	(instance colormaterial
		  :create :name :MediumAquamarine
		  :diffuse #f(0.3984375 0.80078125 0.6640625))
	(instance colormaterial
		  :create :name :aquamarine
		  :diffuse #f(0.49609375 0.99609375 0.828125))
	(instance colormaterial
		  :create :name :DarkGreen
		  :diffuse #f(0 0.390625 0))
	(instance colormaterial
		  :create :name :DarkOliveGreen
		  :diffuse #f(0.33203125 0.41796875 0.18359375))
	(instance colormaterial
		  :create :name :DarkSeaGreen
		  :diffuse #f(0.55859375 0.734375 0.55859375))
	(instance colormaterial
		  :create :name :SeaGreen
		  :diffuse #f(0.1796875 0.54296875 0.33984375))
	(instance colormaterial
		  :create :name :MediumSeaGreen
		  :diffuse #f(0.234375 0.69921875 0.44140625))
	(instance colormaterial
		  :create :name :LightSeaGreen
		  :diffuse #f(0.125 0.6953125 0.6640625))
	(instance colormaterial
		  :create :name :PaleGreen
		  :diffuse #f(0.59375 0.98046875 0.59375))
	(instance colormaterial
		  :create :name :SpringGreen
		  :diffuse #f(0 0.99609375 0.49609375))
	(instance colormaterial
		  :create :name :LawnGreen
		  :diffuse #f(0.484375 0.984375 0))
	(instance colormaterial
		  :create :name :green
		  :diffuse #f(0 0.99609375 0))
	(instance colormaterial
		  :create :name :chartreuse
		  :diffuse #f(0.49609375 0.99609375 0))
	(instance colormaterial
		  :create :name :MediumSpringGreen
		  :diffuse #f(0 0.9765625 0.6015625))
	(instance colormaterial
		  :create :name :GreenYellow
		  :diffuse #f(0.67578125 0.99609375 0.18359375))
	(instance colormaterial
		  :create :name :LimeGreen
		  :diffuse #f(0.1953125 0.80078125 0.1953125))
	(instance colormaterial
		  :create :name :YellowGreen
		  :diffuse #f(0.6015625 0.80078125 0.1953125))
	(instance colormaterial
		  :create :name :ForestGreen
		  :diffuse #f(0.1328125 0.54296875 0.1328125))
	(instance colormaterial
		  :create :name :OliveDrab
		  :diffuse #f(0.41796875 0.5546875 0.13671875))
	(instance colormaterial
		  :create :name :DarkKhaki
		  :diffuse #f(0.73828125 0.71484375 0.41796875))
	(instance colormaterial
		  :create :name :khaki
		  :diffuse #f(0.9375 0.8984375 0.546875))
	(instance colormaterial
		  :create :name :PaleGoldenrod
		  :diffuse #f(0.9296875 0.90625 0.6640625))
	(instance colormaterial
		  :create :name :LightGoldenrodYellow
		  :diffuse #f(0.9765625 0.9765625 0.8203125))
	(instance colormaterial
		  :create :name :LightYellow
		  :diffuse #f(0.99609375 0.99609375 0.875))
	(instance colormaterial
		  :create :name :yellow
		  :diffuse #f(0.99609375 0.99609375 0))
	(instance colormaterial
		  :create :name :gold
		  :diffuse #f(0.99609375 0.83984375 0))
	(instance colormaterial
		  :create :name :LightGoldenrod
		  :diffuse #f(0.9296875 0.86328125 0.5078125))
	(instance colormaterial
		  :create :name :goldenrod
		  :diffuse #f(0.8515625 0.64453125 0.125))
	(instance colormaterial
		  :create :name :DarkGoldenrod
		  :diffuse #f(0.71875 0.5234375 0.04296875))
	(instance colormaterial
		  :create :name :RosyBrown
		  :diffuse #f(0.734375 0.55859375 0.55859375))
	(instance colormaterial
		  :create :name :IndianRed
		  :diffuse #f(0.80078125 0.359375 0.359375))
	(instance colormaterial
		  :create :name :SaddleBrown
		  :diffuse #f(0.54296875 0.26953125 0.07421875))
	(instance colormaterial
		  :create :name :sienna
		  :diffuse #f(0.625 0.3203125 0.17578125))
	(instance colormaterial
		  :create :name :peru
		  :diffuse #f(0.80078125 0.51953125 0.24609375))
	(instance colormaterial
		  :create :name :burlywood
		  :diffuse #f(0.8671875 0.71875 0.52734375))
	(instance colormaterial
		  :create :name :beige
		  :diffuse #f(0.95703125 0.95703125 0.859375))
	(instance colormaterial
		  :create :name :wheat
		  :diffuse #f(0.95703125 0.8671875 0.69921875))
	(instance colormaterial
		  :create :name :SandyBrown
		  :diffuse #f(0.953125 0.640625 0.375))
	(instance colormaterial
		  :create :name :tan
		  :diffuse #f(0.8203125 0.703125 0.546875))
	(instance colormaterial
		  :create :name :chocolate
		  :diffuse #f(0.8203125 0.41015625 0.1171875))
	(instance colormaterial
		  :create :name :firebrick
		  :diffuse #f(0.6953125 0.1328125 0.1328125))
	(instance colormaterial
		  :create :name :brown
		  :diffuse #f(0.64453125 0.1640625 0.1640625))
	(instance colormaterial
		  :create :name :DarkSalmon
		  :diffuse #f(0.91015625 0.5859375 0.4765625))
	(instance colormaterial
		  :create :name :salmon
		  :diffuse #f(0.9765625 0.5 0.4453125))
	(instance colormaterial
		  :create :name :LightSalmon
		  :diffuse #f(0.99609375 0.625 0.4765625))
	(instance colormaterial
		  :create :name :orange
		  :diffuse #f(0.99609375 0.64453125 0))
	(instance colormaterial
		  :create :name :DarkOrange
		  :diffuse #f(0.99609375 0.546875 0))
	(instance colormaterial
		  :create :name :coral
		  :diffuse #f(0.99609375 0.49609375 0.3125))
	(instance colormaterial
		  :create :name :LightCoral
		  :diffuse #f(0.9375 0.5 0.5))
	(instance colormaterial
		  :create :name :tomato
		  :diffuse #f(0.99609375 0.38671875 0.27734375))
	(instance colormaterial
		  :create :name :OrangeRed
		  :diffuse #f(0.99609375 0.26953125 0))
	(instance colormaterial
		  :create :name :red
		  :diffuse #f(0.99609375 0 0))
	(instance colormaterial
		  :create :name :HotPink
		  :diffuse #f(0.99609375 0.41015625 0.703125))
	(instance colormaterial
		  :create :name :DeepPink
		  :diffuse #f(0.99609375 0.078125 0.57421875))
	(instance colormaterial
		  :create :name :pink
		  :diffuse #f(0.99609375 0.75 0.79296875))
	(instance colormaterial
		  :create :name :LightPink
		  :diffuse #f(0.99609375 0.7109375 0.75390625))
	(instance colormaterial
		  :create :name :PaleVioletRed
		  :diffuse #f(0.85546875 0.4375 0.57421875))
	(instance colormaterial
		  :create :name :maroon
		  :diffuse #f(0.6875 0.1875 0.375))
	(instance colormaterial
		  :create :name :MediumVioletRed
		  :diffuse #f(0.77734375 0.08203125 0.51953125))
	(instance colormaterial
		  :create :name :VioletRed
		  :diffuse #f(0.8125 0.125 0.5625))
	(instance colormaterial
		  :create :name :magenta
		  :diffuse #f(0.99609375 0 0.99609375))
	(instance colormaterial
		  :create :name :violet
		  :diffuse #f(0.9296875 0.5078125 0.9296875))
	(instance colormaterial
		  :create :name :plum
		  :diffuse #f(0.86328125 0.625 0.86328125))
	(instance colormaterial
		  :create :name :orchid
		  :diffuse #f(0.8515625 0.4375 0.8359375))
	(instance colormaterial
		  :create :name :MediumOrchid
		  :diffuse #f(0.7265625 0.33203125 0.82421875))
	(instance colormaterial
		  :create :name :DarkOrchid
		  :diffuse #f(0.59765625 0.1953125 0.796875))
	(instance colormaterial
		  :create :name :DarkViolet
		  :diffuse #f(0.578125 0 0.82421875))
	(instance colormaterial
		  :create :name :BlueViolet
		  :diffuse #f(0.5390625 0.16796875 0.8828125))
	(instance colormaterial
		  :create :name :purple
		  :diffuse #f(0.625 0.125 0.9375))
	(instance colormaterial
		  :create :name :MediumPurple
		  :diffuse #f(0.57421875 0.4375 0.85546875))
	(instance colormaterial
		  :create :name :thistle
		  :diffuse #f(0.84375 0.74609375 0.84375))
	(instance colormaterial
		  :create :name :snow1
		  :diffuse #f(0.99609375 0.9765625 0.9765625))
	(instance colormaterial
		  :create :name :snow2
		  :diffuse #f(0.9296875 0.91015625 0.91015625))
	(instance colormaterial
		  :create :name :snow3
		  :diffuse #f(0.80078125 0.78515625 0.78515625))
	(instance colormaterial
		  :create :name :snow4
		  :diffuse #f(0.54296875 0.53515625 0.53515625))
	(instance colormaterial
		  :create :name :seashell1
		  :diffuse #f(0.99609375 0.95703125 0.9296875))
	(instance colormaterial
		  :create :name :seashell2
		  :diffuse #f(0.9296875 0.89453125 0.8671875))
	(instance colormaterial
		  :create :name :seashell3
		  :diffuse #f(0.80078125 0.76953125 0.74609375))
	(instance colormaterial
		  :create :name :seashell4
		  :diffuse #f(0.54296875 0.5234375 0.5078125))
	(instance colormaterial
		  :create :name :AntiqueWhite1
		  :diffuse #f(0.99609375 0.93359375 0.85546875))
	(instance colormaterial
		  :create :name :AntiqueWhite2
		  :diffuse #f(0.9296875 0.87109375 0.796875))
	(instance colormaterial
		  :create :name :AntiqueWhite3
		  :diffuse #f(0.80078125 0.75 0.6875))
	(instance colormaterial
		  :create :name :AntiqueWhite4
		  :diffuse #f(0.54296875 0.51171875 0.46875))
	(instance colormaterial
		  :create :name :bisque1
		  :diffuse #f(0.99609375 0.890625 0.765625))
	(instance colormaterial
		  :create :name :bisque2
		  :diffuse #f(0.9296875 0.83203125 0.71484375))
	(instance colormaterial
		  :create :name :bisque3
		  :diffuse #f(0.80078125 0.71484375 0.6171875))
	(instance colormaterial
		  :create :name :bisque4
		  :diffuse #f(0.54296875 0.48828125 0.41796875))
	(instance colormaterial
		  :create :name :PeachPuff1
		  :diffuse #f(0.99609375 0.8515625 0.72265625))
	(instance colormaterial
		  :create :name :PeachPuff2
		  :diffuse #f(0.9296875 0.79296875 0.67578125))
	(instance colormaterial
		  :create :name :PeachPuff3
		  :diffuse #f(0.80078125 0.68359375 0.58203125))
	(instance colormaterial
		  :create :name :PeachPuff4
		  :diffuse #f(0.54296875 0.46484375 0.39453125))
	(instance colormaterial
		  :create :name :NavajoWhite1
		  :diffuse #f(0.99609375 0.8671875 0.67578125))
	(instance colormaterial
		  :create :name :NavajoWhite2
		  :diffuse #f(0.9296875 0.80859375 0.62890625))
	(instance colormaterial
		  :create :name :NavajoWhite3
		  :diffuse #f(0.80078125 0.69921875 0.54296875))
	(instance colormaterial
		  :create :name :NavajoWhite4
		  :diffuse #f(0.54296875 0.47265625 0.3671875))
	(instance colormaterial
		  :create :name :LemonChiffon1
		  :diffuse #f(0.99609375 0.9765625 0.80078125))
	(instance colormaterial
		  :create :name :LemonChiffon2
		  :diffuse #f(0.9296875 0.91015625 0.74609375))
	(instance colormaterial
		  :create :name :LemonChiffon3
		  :diffuse #f(0.80078125 0.78515625 0.64453125))
	(instance colormaterial
		  :create :name :LemonChiffon4
		  :diffuse #f(0.54296875 0.53515625 0.4375))
	(instance colormaterial
		  :create :name :cornsilk1
		  :diffuse #f(0.99609375 0.96875 0.859375))
	(instance colormaterial
		  :create :name :cornsilk2
		  :diffuse #f(0.9296875 0.90625 0.80078125))
	(instance colormaterial
		  :create :name :cornsilk3
		  :diffuse #f(0.80078125 0.78125 0.69140625))
	(instance colormaterial
		  :create :name :cornsilk4
		  :diffuse #f(0.54296875 0.53125 0.46875))
	(instance colormaterial
		  :create :name :ivory1
		  :diffuse #f(0.99609375 0.99609375 0.9375))
	(instance colormaterial
		  :create :name :ivory2
		  :diffuse #f(0.9296875 0.9296875 0.875))
	(instance colormaterial
		  :create :name :ivory3
		  :diffuse #f(0.80078125 0.80078125 0.75390625))
	(instance colormaterial
		  :create :name :ivory4
		  :diffuse #f(0.54296875 0.54296875 0.51171875))
	(instance colormaterial
		  :create :name :honeydew1
		  :diffuse #f(0.9375 0.99609375 0.9375))
	(instance colormaterial
		  :create :name :honeydew2
		  :diffuse #f(0.875 0.9296875 0.875))
	(instance colormaterial
		  :create :name :honeydew3
		  :diffuse #f(0.75390625 0.80078125 0.75390625))
	(instance colormaterial
		  :create :name :honeydew4
		  :diffuse #f(0.51171875 0.54296875 0.51171875))
	(instance colormaterial
		  :create :name :LavenderBlush1
		  :diffuse #f(0.99609375 0.9375 0.95703125))
	(instance colormaterial
		  :create :name :LavenderBlush2
		  :diffuse #f(0.9296875 0.875 0.89453125))
	(instance colormaterial
		  :create :name :LavenderBlush3
		  :diffuse #f(0.80078125 0.75390625 0.76953125))
	(instance colormaterial
		  :create :name :LavenderBlush4
		  :diffuse #f(0.54296875 0.51171875 0.5234375))
	(instance colormaterial
		  :create :name :MistyRose1
		  :diffuse #f(0.99609375 0.890625 0.87890625))
	(instance colormaterial
		  :create :name :MistyRose2
		  :diffuse #f(0.9296875 0.83203125 0.8203125))
	(instance colormaterial
		  :create :name :MistyRose3
		  :diffuse #f(0.80078125 0.71484375 0.70703125))
	(instance colormaterial
		  :create :name :MistyRose4
		  :diffuse #f(0.54296875 0.48828125 0.48046875))
	(instance colormaterial
		  :create :name :azure1
		  :diffuse #f(0.9375 0.99609375 0.99609375))
	(instance colormaterial
		  :create :name :azure2
		  :diffuse #f(0.875 0.9296875 0.9296875))
	(instance colormaterial
		  :create :name :azure3
		  :diffuse #f(0.75390625 0.80078125 0.80078125))
	(instance colormaterial
		  :create :name :azure4
		  :diffuse #f(0.51171875 0.54296875 0.54296875))
	(instance colormaterial
		  :create :name :SlateBlue1
		  :diffuse #f(0.51171875 0.43359375 0.99609375))
	(instance colormaterial
		  :create :name :SlateBlue2
		  :diffuse #f(0.4765625 0.40234375 0.9296875))
	(instance colormaterial
		  :create :name :SlateBlue3
		  :diffuse #f(0.41015625 0.34765625 0.80078125))
	(instance colormaterial
		  :create :name :SlateBlue4
		  :diffuse #f(0.27734375 0.234375 0.54296875))
	(instance colormaterial
		  :create :name :RoyalBlue1
		  :diffuse #f(0.28125 0.4609375 0.99609375))
	(instance colormaterial
		  :create :name :RoyalBlue2
		  :diffuse #f(0.26171875 0.4296875 0.9296875))
	(instance colormaterial
		  :create :name :RoyalBlue3
		  :diffuse #f(0.2265625 0.37109375 0.80078125))
	(instance colormaterial
		  :create :name :RoyalBlue4
		  :diffuse #f(0.15234375 0.25 0.54296875))
	(instance colormaterial
		  :create :name :blue1
		  :diffuse #f(0 0 0.99609375))
	(instance colormaterial
		  :create :name :blue2
		  :diffuse #f(0 0 0.9296875))
	(instance colormaterial
		  :create :name :blue3
		  :diffuse #f(0 0 0.80078125))
	(instance colormaterial
		  :create :name :blue4
		  :diffuse #f(0 0 0.54296875))
	(instance colormaterial
		  :create :name :DodgerBlue1
		  :diffuse #f(0.1171875 0.5625 0.99609375))
	(instance colormaterial
		  :create :name :DodgerBlue2
		  :diffuse #f(0.109375 0.5234375 0.9296875))
	(instance colormaterial
		  :create :name :DodgerBlue3
		  :diffuse #f(0.09375 0.453125 0.80078125))
	(instance colormaterial
		  :create :name :DodgerBlue4
		  :diffuse #f(0.0625 0.3046875 0.54296875))
	(instance colormaterial
		  :create :name :SteelBlue1
		  :diffuse #f(0.38671875 0.71875 0.99609375))
	(instance colormaterial
		  :create :name :SteelBlue2
		  :diffuse #f(0.359375 0.671875 0.9296875))
	(instance colormaterial
		  :create :name :SteelBlue3
		  :diffuse #f(0.30859375 0.578125 0.80078125))
	(instance colormaterial
		  :create :name :SteelBlue4
		  :diffuse #f(0.2109375 0.390625 0.54296875))
	(instance colormaterial
		  :create :name :DeepSkyBlue1
		  :diffuse #f(0 0.74609375 0.99609375))
	(instance colormaterial
		  :create :name :DeepSkyBlue2
		  :diffuse #f(0 0.6953125 0.9296875))
	(instance colormaterial
		  :create :name :DeepSkyBlue3
		  :diffuse #f(0 0.6015625 0.80078125))
	(instance colormaterial
		  :create :name :DeepSkyBlue4
		  :diffuse #f(0 0.40625 0.54296875))
	(instance colormaterial
		  :create :name :SkyBlue1
		  :diffuse #f(0.52734375 0.8046875 0.99609375))
	(instance colormaterial
		  :create :name :SkyBlue2
		  :diffuse #f(0.4921875 0.75 0.9296875))
	(instance colormaterial
		  :create :name :SkyBlue3
		  :diffuse #f(0.421875 0.6484375 0.80078125))
	(instance colormaterial
		  :create :name :SkyBlue4
		  :diffuse #f(0.2890625 0.4375 0.54296875))
	(instance colormaterial
		  :create :name :LightSkyBlue1
		  :diffuse #f(0.6875 0.8828125 0.99609375))
	(instance colormaterial
		  :create :name :LightSkyBlue2
		  :diffuse #f(0.640625 0.82421875 0.9296875))
	(instance colormaterial
		  :create :name :LightSkyBlue3
		  :diffuse #f(0.55078125 0.7109375 0.80078125))
	(instance colormaterial
		  :create :name :LightSkyBlue4
		  :diffuse #f(0.375 0.48046875 0.54296875))
	(instance colormaterial
		  :create :name :SlateGray1
		  :diffuse #f(0.7734375 0.8828125 0.99609375))
	(instance colormaterial
		  :create :name :SlateGray2
		  :diffuse #f(0.72265625 0.82421875 0.9296875))
	(instance colormaterial
		  :create :name :SlateGray3
		  :diffuse #f(0.62109375 0.7109375 0.80078125))
	(instance colormaterial
		  :create :name :SlateGray4
		  :diffuse #f(0.421875 0.48046875 0.54296875))
	(instance colormaterial
		  :create :name :LightSteelBlue1
		  :diffuse #f(0.7890625 0.87890625 0.99609375))
	(instance colormaterial
		  :create :name :LightSteelBlue2
		  :diffuse #f(0.734375 0.8203125 0.9296875))
	(instance colormaterial
		  :create :name :LightSteelBlue3
		  :diffuse #f(0.6328125 0.70703125 0.80078125))
	(instance colormaterial
		  :create :name :LightSteelBlue4
		  :diffuse #f(0.4296875 0.48046875 0.54296875))
	(instance colormaterial
		  :create :name :LightBlue1
		  :diffuse #f(0.74609375 0.93359375 0.99609375))
	(instance colormaterial
		  :create :name :LightBlue2
		  :diffuse #f(0.6953125 0.87109375 0.9296875))
	(instance colormaterial
		  :create :name :LightBlue3
		  :diffuse #f(0.6015625 0.75 0.80078125))
	(instance colormaterial
		  :create :name :LightBlue4
		  :diffuse #f(0.40625 0.51171875 0.54296875))
	(instance colormaterial
		  :create :name :LightCyan1
		  :diffuse #f(0.875 0.99609375 0.99609375))
	(instance colormaterial
		  :create :name :LightCyan2
		  :diffuse #f(0.81640625 0.9296875 0.9296875))
	(instance colormaterial
		  :create :name :LightCyan3
		  :diffuse #f(0.703125 0.80078125 0.80078125))
	(instance colormaterial
		  :create :name :LightCyan4
		  :diffuse #f(0.4765625 0.54296875 0.54296875))
	(instance colormaterial
		  :create :name :PaleTurquoise1
		  :diffuse #f(0.73046875 0.99609375 0.99609375))
	(instance colormaterial
		  :create :name :PaleTurquoise2
		  :diffuse #f(0.6796875 0.9296875 0.9296875))
	(instance colormaterial
		  :create :name :PaleTurquoise3
		  :diffuse #f(0.5859375 0.80078125 0.80078125))
	(instance colormaterial
		  :create :name :PaleTurquoise4
		  :diffuse #f(0.3984375 0.54296875 0.54296875))
	(instance colormaterial
		  :create :name :CadetBlue1
		  :diffuse #f(0.59375 0.95703125 0.99609375))
	(instance colormaterial
		  :create :name :CadetBlue2
		  :diffuse #f(0.5546875 0.89453125 0.9296875))
	(instance colormaterial
		  :create :name :CadetBlue3
		  :diffuse #f(0.4765625 0.76953125 0.80078125))
	(instance colormaterial
		  :create :name :CadetBlue4
		  :diffuse #f(0.32421875 0.5234375 0.54296875))
	(instance colormaterial
		  :create :name :turquoise1
		  :diffuse #f(0 0.95703125 0.99609375))
	(instance colormaterial
		  :create :name :turquoise2
		  :diffuse #f(0 0.89453125 0.9296875))
	(instance colormaterial
		  :create :name :turquoise3
		  :diffuse #f(0 0.76953125 0.80078125))
	(instance colormaterial
		  :create :name :turquoise4
		  :diffuse #f(0 0.5234375 0.54296875))
	(instance colormaterial
		  :create :name :cyan1
		  :diffuse #f(0 0.99609375 0.99609375))
	(instance colormaterial
		  :create :name :cyan2
		  :diffuse #f(0 0.9296875 0.9296875))
	(instance colormaterial
		  :create :name :cyan3
		  :diffuse #f(0 0.80078125 0.80078125))
	(instance colormaterial
		  :create :name :cyan4
		  :diffuse #f(0 0.54296875 0.54296875))
	(instance colormaterial
		  :create :name :DarkSlateGray1
		  :diffuse #f(0.58984375 0.99609375 0.99609375))
	(instance colormaterial
		  :create :name :DarkSlateGray2
		  :diffuse #f(0.55078125 0.9296875 0.9296875))
	(instance colormaterial
		  :create :name :DarkSlateGray3
		  :diffuse #f(0.47265625 0.80078125 0.80078125))
	(instance colormaterial
		  :create :name :DarkSlateGray4
		  :diffuse #f(0.3203125 0.54296875 0.54296875))
	(instance colormaterial
		  :create :name :aquamarine1
		  :diffuse #f(0.49609375 0.99609375 0.828125))
	(instance colormaterial
		  :create :name :aquamarine2
		  :diffuse #f(0.4609375 0.9296875 0.7734375))
	(instance colormaterial
		  :create :name :aquamarine3
		  :diffuse #f(0.3984375 0.80078125 0.6640625))
	(instance colormaterial
		  :create :name :aquamarine4
		  :diffuse #f(0.26953125 0.54296875 0.453125))
	(instance colormaterial
		  :create :name :DarkSeaGreen1
		  :diffuse #f(0.75390625 0.99609375 0.75390625))
	(instance colormaterial
		  :create :name :DarkSeaGreen2
		  :diffuse #f(0.703125 0.9296875 0.703125))
	(instance colormaterial
		  :create :name :DarkSeaGreen3
		  :diffuse #f(0.60546875 0.80078125 0.60546875))
	(instance colormaterial
		  :create :name :DarkSeaGreen4
		  :diffuse #f(0.41015625 0.54296875 0.41015625))
	(instance colormaterial
		  :create :name :SeaGreen1
		  :diffuse #f(0.328125 0.99609375 0.62109375))
	(instance colormaterial
		  :create :name :SeaGreen2
		  :diffuse #f(0.3046875 0.9296875 0.578125))
	(instance colormaterial
		  :create :name :SeaGreen3
		  :diffuse #f(0.26171875 0.80078125 0.5))
	(instance colormaterial
		  :create :name :SeaGreen4
		  :diffuse #f(0.1796875 0.54296875 0.33984375))
	(instance colormaterial
		  :create :name :PaleGreen1
		  :diffuse #f(0.6015625 0.99609375 0.6015625))
	(instance colormaterial
		  :create :name :PaleGreen2
		  :diffuse #f(0.5625 0.9296875 0.5625))
	(instance colormaterial
		  :create :name :PaleGreen3
		  :diffuse #f(0.484375 0.80078125 0.484375))
	(instance colormaterial
		  :create :name :PaleGreen4
		  :diffuse #f(0.328125 0.54296875 0.328125))
	(instance colormaterial
		  :create :name :SpringGreen1
		  :diffuse #f(0 0.99609375 0.49609375))
	(instance colormaterial
		  :create :name :SpringGreen2
		  :diffuse #f(0 0.9296875 0.4609375))
	(instance colormaterial
		  :create :name :SpringGreen3
		  :diffuse #f(0 0.80078125 0.3984375))
	(instance colormaterial
		  :create :name :SpringGreen4
		  :diffuse #f(0 0.54296875 0.26953125))
	(instance colormaterial
		  :create :name :green1
		  :diffuse #f(0 0.99609375 0))
	(instance colormaterial
		  :create :name :green2
		  :diffuse #f(0 0.9296875 0))
	(instance colormaterial
		  :create :name :green3
		  :diffuse #f(0 0.80078125 0))
	(instance colormaterial
		  :create :name :green4
		  :diffuse #f(0 0.54296875 0))
	(instance colormaterial
		  :create :name :chartreuse1
		  :diffuse #f(0.49609375 0.99609375 0))
	(instance colormaterial
		  :create :name :chartreuse2
		  :diffuse #f(0.4609375 0.9296875 0))
	(instance colormaterial
		  :create :name :chartreuse3
		  :diffuse #f(0.3984375 0.80078125 0))
	(instance colormaterial
		  :create :name :chartreuse4
		  :diffuse #f(0.26953125 0.54296875 0))
	(instance colormaterial
		  :create :name :OliveDrab1
		  :diffuse #f(0.75 0.99609375 0.2421875))
	(instance colormaterial
		  :create :name :OliveDrab2
		  :diffuse #f(0.69921875 0.9296875 0.2265625))
	(instance colormaterial
		  :create :name :OliveDrab3
		  :diffuse #f(0.6015625 0.80078125 0.1953125))
	(instance colormaterial
		  :create :name :OliveDrab4
		  :diffuse #f(0.41015625 0.54296875 0.1328125))
	(instance colormaterial
		  :create :name :DarkOliveGreen1
		  :diffuse #f(0.7890625 0.99609375 0.4375))
	(instance colormaterial
		  :create :name :DarkOliveGreen2
		  :diffuse #f(0.734375 0.9296875 0.40625))
	(instance colormaterial
		  :create :name :DarkOliveGreen3
		  :diffuse #f(0.6328125 0.80078125 0.3515625))
	(instance colormaterial
		  :create :name :DarkOliveGreen4
		  :diffuse #f(0.4296875 0.54296875 0.23828125))
	(instance colormaterial
		  :create :name :khaki1
		  :diffuse #f(0.99609375 0.9609375 0.55859375))
	(instance colormaterial
		  :create :name :khaki2
		  :diffuse #f(0.9296875 0.8984375 0.51953125))
	(instance colormaterial
		  :create :name :khaki3
		  :diffuse #f(0.80078125 0.7734375 0.44921875))
	(instance colormaterial
		  :create :name :khaki4
		  :diffuse #f(0.54296875 0.5234375 0.3046875))
	(instance colormaterial
		  :create :name :LightGoldenrod1
		  :diffuse #f(0.99609375 0.921875 0.54296875))
	(instance colormaterial
		  :create :name :LightGoldenrod2
		  :diffuse #f(0.9296875 0.859375 0.5078125))
	(instance colormaterial
		  :create :name :LightGoldenrod3
		  :diffuse #f(0.80078125 0.7421875 0.4375))
	(instance colormaterial
		  :create :name :LightGoldenrod4
		  :diffuse #f(0.54296875 0.50390625 0.296875))
	(instance colormaterial
		  :create :name :LightYellow1
		  :diffuse #f(0.99609375 0.99609375 0.875))
	(instance colormaterial
		  :create :name :LightYellow2
		  :diffuse #f(0.9296875 0.9296875 0.81640625))
	(instance colormaterial
		  :create :name :LightYellow3
		  :diffuse #f(0.80078125 0.80078125 0.703125))
	(instance colormaterial
		  :create :name :LightYellow4
		  :diffuse #f(0.54296875 0.54296875 0.4765625))
	(instance colormaterial
		  :create :name :yellow1
		  :diffuse #f(0.99609375 0.99609375 0))
	(instance colormaterial
		  :create :name :yellow2
		  :diffuse #f(0.9296875 0.9296875 0))
	(instance colormaterial
		  :create :name :yellow3
		  :diffuse #f(0.80078125 0.80078125 0))
	(instance colormaterial
		  :create :name :yellow4
		  :diffuse #f(0.54296875 0.54296875 0))
	(instance colormaterial
		  :create :name :gold1
		  :diffuse #f(0.99609375 0.83984375 0))
	(instance colormaterial
		  :create :name :gold2
		  :diffuse #f(0.9296875 0.78515625 0))
	(instance colormaterial
		  :create :name :gold3
		  :diffuse #f(0.80078125 0.67578125 0))
	(instance colormaterial
		  :create :name :gold4
		  :diffuse #f(0.54296875 0.45703125 0))
	(instance colormaterial
		  :create :name :goldenrod1
		  :diffuse #f(0.99609375 0.75390625 0.14453125))
	(instance colormaterial
		  :create :name :goldenrod2
		  :diffuse #f(0.9296875 0.703125 0.1328125))
	(instance colormaterial
		  :create :name :goldenrod3
		  :diffuse #f(0.80078125 0.60546875 0.11328125))
	(instance colormaterial
		  :create :name :goldenrod4
		  :diffuse #f(0.54296875 0.41015625 0.078125))
	(instance colormaterial
		  :create :name :DarkGoldenrod1
		  :diffuse #f(0.99609375 0.72265625 0.05859375))
	(instance colormaterial
		  :create :name :DarkGoldenrod2
		  :diffuse #f(0.9296875 0.67578125 0.0546875))
	(instance colormaterial
		  :create :name :DarkGoldenrod3
		  :diffuse #f(0.80078125 0.58203125 0.046875))
	(instance colormaterial
		  :create :name :DarkGoldenrod4
		  :diffuse #f(0.54296875 0.39453125 0.03125))
	(instance colormaterial
		  :create :name :RosyBrown1
		  :diffuse #f(0.99609375 0.75390625 0.75390625))
	(instance colormaterial
		  :create :name :RosyBrown2
		  :diffuse #f(0.9296875 0.703125 0.703125))
	(instance colormaterial
		  :create :name :RosyBrown3
		  :diffuse #f(0.80078125 0.60546875 0.60546875))
	(instance colormaterial
		  :create :name :RosyBrown4
		  :diffuse #f(0.54296875 0.41015625 0.41015625))
	(instance colormaterial
		  :create :name :IndianRed1
		  :diffuse #f(0.99609375 0.4140625 0.4140625))
	(instance colormaterial
		  :create :name :IndianRed2
		  :diffuse #f(0.9296875 0.38671875 0.38671875))
	(instance colormaterial
		  :create :name :IndianRed3
		  :diffuse #f(0.80078125 0.33203125 0.33203125))
	(instance colormaterial
		  :create :name :IndianRed4
		  :diffuse #f(0.54296875 0.2265625 0.2265625))
	(instance colormaterial
		  :create :name :sienna1
		  :diffuse #f(0.99609375 0.5078125 0.27734375))
	(instance colormaterial
		  :create :name :sienna2
		  :diffuse #f(0.9296875 0.47265625 0.2578125))
	(instance colormaterial
		  :create :name :sienna3
		  :diffuse #f(0.80078125 0.40625 0.22265625))
	(instance colormaterial
		  :create :name :sienna4
		  :diffuse #f(0.54296875 0.27734375 0.1484375))
	(instance colormaterial
		  :create :name :burlywood1
		  :diffuse #f(0.99609375 0.82421875 0.60546875))
	(instance colormaterial
		  :create :name :burlywood2
		  :diffuse #f(0.9296875 0.76953125 0.56640625))
	(instance colormaterial
		  :create :name :burlywood3
		  :diffuse #f(0.80078125 0.6640625 0.48828125))
	(instance colormaterial
		  :create :name :burlywood4
		  :diffuse #f(0.54296875 0.44921875 0.33203125))
	(instance colormaterial
		  :create :name :wheat1
		  :diffuse #f(0.99609375 0.90234375 0.7265625))
	(instance colormaterial
		  :create :name :wheat2
		  :diffuse #f(0.9296875 0.84375 0.6796875))
	(instance colormaterial
		  :create :name :wheat3
		  :diffuse #f(0.80078125 0.7265625 0.5859375))
	(instance colormaterial
		  :create :name :wheat4
		  :diffuse #f(0.54296875 0.4921875 0.3984375))
	(instance colormaterial
		  :create :name :tan1
		  :diffuse #f(0.99609375 0.64453125 0.30859375))
	(instance colormaterial
		  :create :name :tan2
		  :diffuse #f(0.9296875 0.6015625 0.28515625))
	(instance colormaterial
		  :create :name :tan3
		  :diffuse #f(0.80078125 0.51953125 0.24609375))
	(instance colormaterial
		  :create :name :tan4
		  :diffuse #f(0.54296875 0.3515625 0.16796875))
	(instance colormaterial
		  :create :name :chocolate1
		  :diffuse #f(0.99609375 0.49609375 0.140625))
	(instance colormaterial
		  :create :name :chocolate2
		  :diffuse #f(0.9296875 0.4609375 0.12890625))
	(instance colormaterial
		  :create :name :chocolate3
		  :diffuse #f(0.80078125 0.3984375 0.11328125))
	(instance colormaterial
		  :create :name :chocolate4
		  :diffuse #f(0.54296875 0.26953125 0.07421875))
	(instance colormaterial
		  :create :name :firebrick1
		  :diffuse #f(0.99609375 0.1875 0.1875))
	(instance colormaterial
		  :create :name :firebrick2
		  :diffuse #f(0.9296875 0.171875 0.171875))
	(instance colormaterial
		  :create :name :firebrick3
		  :diffuse #f(0.80078125 0.1484375 0.1484375))
	(instance colormaterial
		  :create :name :firebrick4
		  :diffuse #f(0.54296875 0.1015625 0.1015625))
	(instance colormaterial
		  :create :name :brown1
		  :diffuse #f(0.99609375 0.25 0.25))
	(instance colormaterial
		  :create :name :brown2
		  :diffuse #f(0.9296875 0.23046875 0.23046875))
	(instance colormaterial
		  :create :name :brown3
		  :diffuse #f(0.80078125 0.19921875 0.19921875))
	(instance colormaterial
		  :create :name :brown4
		  :diffuse #f(0.54296875 0.13671875 0.13671875))
	(instance colormaterial
		  :create :name :salmon1
		  :diffuse #f(0.99609375 0.546875 0.41015625))
	(instance colormaterial
		  :create :name :salmon2
		  :diffuse #f(0.9296875 0.5078125 0.3828125))
	(instance colormaterial
		  :create :name :salmon3
		  :diffuse #f(0.80078125 0.4375 0.328125))
	(instance colormaterial
		  :create :name :salmon4
		  :diffuse #f(0.54296875 0.296875 0.22265625))
	(instance colormaterial
		  :create :name :LightSalmon1
		  :diffuse #f(0.99609375 0.625 0.4765625))
	(instance colormaterial
		  :create :name :LightSalmon2
		  :diffuse #f(0.9296875 0.58203125 0.4453125))
	(instance colormaterial
		  :create :name :LightSalmon3
		  :diffuse #f(0.80078125 0.50390625 0.3828125))
	(instance colormaterial
		  :create :name :LightSalmon4
		  :diffuse #f(0.54296875 0.33984375 0.2578125))
	(instance colormaterial
		  :create :name :orange1
		  :diffuse #f(0.99609375 0.64453125 0))
	(instance colormaterial
		  :create :name :orange2
		  :diffuse #f(0.9296875 0.6015625 0))
	(instance colormaterial
		  :create :name :orange3
		  :diffuse #f(0.80078125 0.51953125 0))
	(instance colormaterial
		  :create :name :orange4
		  :diffuse #f(0.54296875 0.3515625 0))
	(instance colormaterial
		  :create :name :DarkOrange1
		  :diffuse #f(0.99609375 0.49609375 0))
	(instance colormaterial
		  :create :name :DarkOrange2
		  :diffuse #f(0.9296875 0.4609375 0))
	(instance colormaterial
		  :create :name :DarkOrange3
		  :diffuse #f(0.80078125 0.3984375 0))
	(instance colormaterial
		  :create :name :DarkOrange4
		  :diffuse #f(0.54296875 0.26953125 0))
	(instance colormaterial
		  :create :name :coral1
		  :diffuse #f(0.99609375 0.4453125 0.3359375))
	(instance colormaterial
		  :create :name :coral2
		  :diffuse #f(0.9296875 0.4140625 0.3125))
	(instance colormaterial
		  :create :name :coral3
		  :diffuse #f(0.80078125 0.35546875 0.26953125))
	(instance colormaterial
		  :create :name :coral4
		  :diffuse #f(0.54296875 0.2421875 0.18359375))
	(instance colormaterial
		  :create :name :tomato1
		  :diffuse #f(0.99609375 0.38671875 0.27734375))
	(instance colormaterial
		  :create :name :tomato2
		  :diffuse #f(0.9296875 0.359375 0.2578125))
	(instance colormaterial
		  :create :name :tomato3
		  :diffuse #f(0.80078125 0.30859375 0.22265625))
	(instance colormaterial
		  :create :name :tomato4
		  :diffuse #f(0.54296875 0.2109375 0.1484375))
	(instance colormaterial
		  :create :name :OrangeRed1
		  :diffuse #f(0.99609375 0.26953125 0))
	(instance colormaterial
		  :create :name :OrangeRed2
		  :diffuse #f(0.9296875 0.25 0))
	(instance colormaterial
		  :create :name :OrangeRed3
		  :diffuse #f(0.80078125 0.21484375 0))
	(instance colormaterial
		  :create :name :OrangeRed4
		  :diffuse #f(0.54296875 0.14453125 0))
	(instance colormaterial
		  :create :name :red1
		  :diffuse #f(0.99609375 0 0))
	(instance colormaterial
		  :create :name :red2
		  :diffuse #f(0.9296875 0 0))
	(instance colormaterial
		  :create :name :red3
		  :diffuse #f(0.80078125 0 0))
	(instance colormaterial
		  :create :name :red4
		  :diffuse #f(0.54296875 0 0))
	(instance colormaterial
		  :create :name :DeepPink1
		  :diffuse #f(0.99609375 0.078125 0.57421875))
	(instance colormaterial
		  :create :name :DeepPink2
		  :diffuse #f(0.9296875 0.0703125 0.53515625))
	(instance colormaterial
		  :create :name :DeepPink3
		  :diffuse #f(0.80078125 0.0625 0.4609375))
	(instance colormaterial
		  :create :name :DeepPink4
		  :diffuse #f(0.54296875 0.0390625 0.3125))
	(instance colormaterial
		  :create :name :HotPink1
		  :diffuse #f(0.99609375 0.4296875 0.703125))
	(instance colormaterial
		  :create :name :HotPink2
		  :diffuse #f(0.9296875 0.4140625 0.65234375))
	(instance colormaterial
		  :create :name :HotPink3
		  :diffuse #f(0.80078125 0.375 0.5625))
	(instance colormaterial
		  :create :name :HotPink4
		  :diffuse #f(0.54296875 0.2265625 0.3828125))
	(instance colormaterial
		  :create :name :pink1
		  :diffuse #f(0.99609375 0.70703125 0.76953125))
	(instance colormaterial
		  :create :name :pink2
		  :diffuse #f(0.9296875 0.66015625 0.71875))
	(instance colormaterial
		  :create :name :pink3
		  :diffuse #f(0.80078125 0.56640625 0.6171875))
	(instance colormaterial
		  :create :name :pink4
		  :diffuse #f(0.54296875 0.38671875 0.421875))
	(instance colormaterial
		  :create :name :LightPink1
		  :diffuse #f(0.99609375 0.6796875 0.72265625))
	(instance colormaterial
		  :create :name :LightPink2
		  :diffuse #f(0.9296875 0.6328125 0.67578125))
	(instance colormaterial
		  :create :name :LightPink3
		  :diffuse #f(0.80078125 0.546875 0.58203125))
	(instance colormaterial
		  :create :name :LightPink4
		  :diffuse #f(0.54296875 0.37109375 0.39453125))
	(instance colormaterial
		  :create :name :PaleVioletRed1
		  :diffuse #f(0.99609375 0.5078125 0.66796875))
	(instance colormaterial
		  :create :name :PaleVioletRed2
		  :diffuse #f(0.9296875 0.47265625 0.62109375))
	(instance colormaterial
		  :create :name :PaleVioletRed3
		  :diffuse #f(0.80078125 0.40625 0.53515625))
	(instance colormaterial
		  :create :name :PaleVioletRed4
		  :diffuse #f(0.54296875 0.27734375 0.36328125))
	(instance colormaterial
		  :create :name :maroon1
		  :diffuse #f(0.99609375 0.203125 0.69921875))
	(instance colormaterial
		  :create :name :maroon2
		  :diffuse #f(0.9296875 0.1875 0.65234375))
	(instance colormaterial
		  :create :name :maroon3
		  :diffuse #f(0.80078125 0.16015625 0.5625))
	(instance colormaterial
		  :create :name :maroon4
		  :diffuse #f(0.54296875 0.109375 0.3828125))
	(instance colormaterial
		  :create :name :VioletRed1
		  :diffuse #f(0.99609375 0.2421875 0.5859375))
	(instance colormaterial
		  :create :name :VioletRed2
		  :diffuse #f(0.9296875 0.2265625 0.546875))
	(instance colormaterial
		  :create :name :VioletRed3
		  :diffuse #f(0.80078125 0.1953125 0.46875))
	(instance colormaterial
		  :create :name :VioletRed4
		  :diffuse #f(0.54296875 0.1328125 0.3203125))
	(instance colormaterial
		  :create :name :magenta1
		  :diffuse #f(0.99609375 0 0.99609375))
	(instance colormaterial
		  :create :name :magenta2
		  :diffuse #f(0.9296875 0 0.9296875))
	(instance colormaterial
		  :create :name :magenta3
		  :diffuse #f(0.80078125 0 0.80078125))
	(instance colormaterial
		  :create :name :magenta4
		  :diffuse #f(0.54296875 0 0.54296875))
	(instance colormaterial
		  :create :name :orchid1
		  :diffuse #f(0.99609375 0.51171875 0.9765625))
	(instance colormaterial
		  :create :name :orchid2
		  :diffuse #f(0.9296875 0.4765625 0.91015625))
	(instance colormaterial
		  :create :name :orchid3
		  :diffuse #f(0.80078125 0.41015625 0.78515625))
	(instance colormaterial
		  :create :name :orchid4
		  :diffuse #f(0.54296875 0.27734375 0.53515625))
	(instance colormaterial
		  :create :name :plum1
		  :diffuse #f(0.99609375 0.73046875 0.99609375))
	(instance colormaterial
		  :create :name :plum2
		  :diffuse #f(0.9296875 0.6796875 0.9296875))
	(instance colormaterial
		  :create :name :plum3
		  :diffuse #f(0.80078125 0.5859375 0.80078125))
	(instance colormaterial
		  :create :name :plum4
		  :diffuse #f(0.54296875 0.3984375 0.54296875))
	(instance colormaterial
		  :create :name :MediumOrchid1
		  :diffuse #f(0.875 0.3984375 0.99609375))
	(instance colormaterial
		  :create :name :MediumOrchid2
		  :diffuse #f(0.81640625 0.37109375 0.9296875))
	(instance colormaterial
		  :create :name :MediumOrchid3
		  :diffuse #f(0.703125 0.3203125 0.80078125))
	(instance colormaterial
		  :create :name :MediumOrchid4
		  :diffuse #f(0.4765625 0.21484375 0.54296875))
	(instance colormaterial
		  :create :name :DarkOrchid1
		  :diffuse #f(0.74609375 0.2421875 0.99609375))
	(instance colormaterial
		  :create :name :DarkOrchid2
		  :diffuse #f(0.6953125 0.2265625 0.9296875))
	(instance colormaterial
		  :create :name :DarkOrchid3
		  :diffuse #f(0.6015625 0.1953125 0.80078125))
	(instance colormaterial
		  :create :name :DarkOrchid4
		  :diffuse #f(0.40625 0.1328125 0.54296875))
	(instance colormaterial
		  :create :name :purple1
		  :diffuse #f(0.60546875 0.1875 0.99609375))
	(instance colormaterial
		  :create :name :purple2
		  :diffuse #f(0.56640625 0.171875 0.9296875))
	(instance colormaterial
		  :create :name :purple3
		  :diffuse #f(0.48828125 0.1484375 0.80078125))
	(instance colormaterial
		  :create :name :purple4
		  :diffuse #f(0.33203125 0.1015625 0.54296875))
	(instance colormaterial
		  :create :name :MediumPurple1
		  :diffuse #f(0.66796875 0.5078125 0.99609375))
	(instance colormaterial
		  :create :name :MediumPurple2
		  :diffuse #f(0.62109375 0.47265625 0.9296875))
	(instance colormaterial
		  :create :name :MediumPurple3
		  :diffuse #f(0.53515625 0.40625 0.80078125))
	(instance colormaterial
		  :create :name :MediumPurple4
		  :diffuse #f(0.36328125 0.27734375 0.54296875))
	(instance colormaterial
		  :create :name :thistle1
		  :diffuse #f(0.99609375 0.87890625 0.99609375))
	(instance colormaterial
		  :create :name :thistle2
		  :diffuse #f(0.9296875 0.8203125 0.9296875))
	(instance colormaterial
		  :create :name :thistle3
		  :diffuse #f(0.80078125 0.70703125 0.80078125))
	(instance colormaterial
		  :create :name :thistle4
		  :diffuse #f(0.54296875 0.48046875 0.54296875))
	(instance colormaterial
		  :create :name :gray0
		  :diffuse #f(0 0 0))
	(instance colormaterial
		  :create :name :grey0
		  :diffuse #f(0 0 0))
	(instance colormaterial
		  :create :name :gray1
		  :diffuse #f(0.01171875 0.01171875 0.01171875))
	(instance colormaterial
		  :create :name :grey1
		  :diffuse #f(0.01171875 0.01171875 0.01171875))
	(instance colormaterial
		  :create :name :gray2
		  :diffuse #f(0.01953125 0.01953125 0.01953125))
	(instance colormaterial
		  :create :name :grey2
		  :diffuse #f(0.01953125 0.01953125 0.01953125))
	(instance colormaterial
		  :create :name :gray3
		  :diffuse #f(0.03125 0.03125 0.03125))
	(instance colormaterial
		  :create :name :grey3
		  :diffuse #f(0.03125 0.03125 0.03125))
	(instance colormaterial
		  :create :name :gray4
		  :diffuse #f(0.0390625 0.0390625 0.0390625))
	(instance colormaterial
		  :create :name :grey4
		  :diffuse #f(0.0390625 0.0390625 0.0390625))
	(instance colormaterial
		  :create :name :gray5
		  :diffuse #f(0.05078125 0.05078125 0.05078125))
	(instance colormaterial
		  :create :name :grey5
		  :diffuse #f(0.05078125 0.05078125 0.05078125))
	(instance colormaterial
		  :create :name :gray6
		  :diffuse #f(0.05859375 0.05859375 0.05859375))
	(instance colormaterial
		  :create :name :grey6
		  :diffuse #f(0.05859375 0.05859375 0.05859375))
	(instance colormaterial
		  :create :name :gray7
		  :diffuse #f(0.0703125 0.0703125 0.0703125))
	(instance colormaterial
		  :create :name :grey7
		  :diffuse #f(0.0703125 0.0703125 0.0703125))
	(instance colormaterial
		  :create :name :gray8
		  :diffuse #f(0.078125 0.078125 0.078125))
	(instance colormaterial
		  :create :name :grey8
		  :diffuse #f(0.078125 0.078125 0.078125))
	(instance colormaterial
		  :create :name :gray9
		  :diffuse #f(0.08984375 0.08984375 0.08984375))
	(instance colormaterial
		  :create :name :grey9
		  :diffuse #f(0.08984375 0.08984375 0.08984375))
	(instance colormaterial
		  :create :name :gray10
		  :diffuse #f(0.1015625 0.1015625 0.1015625))
	(instance colormaterial
		  :create :name :grey10
		  :diffuse #f(0.1015625 0.1015625 0.1015625))
	(instance colormaterial
		  :create :name :gray11
		  :diffuse #f(0.109375 0.109375 0.109375))
	(instance colormaterial
		  :create :name :grey11
		  :diffuse #f(0.109375 0.109375 0.109375))
	(instance colormaterial
		  :create :name :gray12
		  :diffuse #f(0.12109375 0.12109375 0.12109375))
	(instance colormaterial
		  :create :name :grey12
		  :diffuse #f(0.12109375 0.12109375 0.12109375))
	(instance colormaterial
		  :create :name :gray13
		  :diffuse #f(0.12890625 0.12890625 0.12890625))
	(instance colormaterial
		  :create :name :grey13
		  :diffuse #f(0.12890625 0.12890625 0.12890625))
	(instance colormaterial
		  :create :name :gray14
		  :diffuse #f(0.140625 0.140625 0.140625))
	(instance colormaterial
		  :create :name :grey14
		  :diffuse #f(0.140625 0.140625 0.140625))
	(instance colormaterial
		  :create :name :gray15
		  :diffuse #f(0.1484375 0.1484375 0.1484375))
	(instance colormaterial
		  :create :name :grey15
		  :diffuse #f(0.1484375 0.1484375 0.1484375))
	(instance colormaterial
		  :create :name :gray16
		  :diffuse #f(0.16015625 0.16015625 0.16015625))
	(instance colormaterial
		  :create :name :grey16
		  :diffuse #f(0.16015625 0.16015625 0.16015625))
	(instance colormaterial
		  :create :name :gray17
		  :diffuse #f(0.16796875 0.16796875 0.16796875))
	(instance colormaterial
		  :create :name :grey17
		  :diffuse #f(0.16796875 0.16796875 0.16796875))
	(instance colormaterial
		  :create :name :gray18
		  :diffuse #f(0.1796875 0.1796875 0.1796875))
	(instance colormaterial
		  :create :name :grey18
		  :diffuse #f(0.1796875 0.1796875 0.1796875))
	(instance colormaterial
		  :create :name :gray19
		  :diffuse #f(0.1875 0.1875 0.1875))
	(instance colormaterial
		  :create :name :grey19
		  :diffuse #f(0.1875 0.1875 0.1875))
	(instance colormaterial
		  :create :name :gray20
		  :diffuse #f(0.19921875 0.19921875 0.19921875))
	(instance colormaterial
		  :create :name :grey20
		  :diffuse #f(0.19921875 0.19921875 0.19921875))
	(instance colormaterial
		  :create :name :gray21
		  :diffuse #f(0.2109375 0.2109375 0.2109375))
	(instance colormaterial
		  :create :name :grey21
		  :diffuse #f(0.2109375 0.2109375 0.2109375))
	(instance colormaterial
		  :create :name :gray22
		  :diffuse #f(0.21875 0.21875 0.21875))
	(instance colormaterial
		  :create :name :grey22
		  :diffuse #f(0.21875 0.21875 0.21875))
	(instance colormaterial
		  :create :name :gray23
		  :diffuse #f(0.23046875 0.23046875 0.23046875))
	(instance colormaterial
		  :create :name :grey23
		  :diffuse #f(0.23046875 0.23046875 0.23046875))
	(instance colormaterial
		  :create :name :gray24
		  :diffuse #f(0.23828125 0.23828125 0.23828125))
	(instance colormaterial
		  :create :name :grey24
		  :diffuse #f(0.23828125 0.23828125 0.23828125))
	(instance colormaterial
		  :create :name :gray25
		  :diffuse #f(0.25 0.25 0.25))
	(instance colormaterial
		  :create :name :grey25
		  :diffuse #f(0.25 0.25 0.25))
	(instance colormaterial
		  :create :name :gray26
		  :diffuse #f(0.2578125 0.2578125 0.2578125))
	(instance colormaterial
		  :create :name :grey26
		  :diffuse #f(0.2578125 0.2578125 0.2578125))
	(instance colormaterial
		  :create :name :gray27
		  :diffuse #f(0.26953125 0.26953125 0.26953125))
	(instance colormaterial
		  :create :name :grey27
		  :diffuse #f(0.26953125 0.26953125 0.26953125))
	(instance colormaterial
		  :create :name :gray28
		  :diffuse #f(0.27734375 0.27734375 0.27734375))
	(instance colormaterial
		  :create :name :grey28
		  :diffuse #f(0.27734375 0.27734375 0.27734375))
	(instance colormaterial
		  :create :name :gray29
		  :diffuse #f(0.2890625 0.2890625 0.2890625))
	(instance colormaterial
		  :create :name :grey29
		  :diffuse #f(0.2890625 0.2890625 0.2890625))
	(instance colormaterial
		  :create :name :gray30
		  :diffuse #f(0.30078125 0.30078125 0.30078125))
	(instance colormaterial
		  :create :name :grey30
		  :diffuse #f(0.30078125 0.30078125 0.30078125))
	(instance colormaterial
		  :create :name :gray31
		  :diffuse #f(0.30859375 0.30859375 0.30859375))
	(instance colormaterial
		  :create :name :grey31
		  :diffuse #f(0.30859375 0.30859375 0.30859375))
	(instance colormaterial
		  :create :name :gray32
		  :diffuse #f(0.3203125 0.3203125 0.3203125))
	(instance colormaterial
		  :create :name :grey32
		  :diffuse #f(0.3203125 0.3203125 0.3203125))
	(instance colormaterial
		  :create :name :gray33
		  :diffuse #f(0.328125 0.328125 0.328125))
	(instance colormaterial
		  :create :name :grey33
		  :diffuse #f(0.328125 0.328125 0.328125))
	(instance colormaterial
		  :create :name :gray34
		  :diffuse #f(0.33984375 0.33984375 0.33984375))
	(instance colormaterial
		  :create :name :grey34
		  :diffuse #f(0.33984375 0.33984375 0.33984375))
	(instance colormaterial
		  :create :name :gray35
		  :diffuse #f(0.34765625 0.34765625 0.34765625))
	(instance colormaterial
		  :create :name :grey35
		  :diffuse #f(0.34765625 0.34765625 0.34765625))
	(instance colormaterial
		  :create :name :gray36
		  :diffuse #f(0.359375 0.359375 0.359375))
	(instance colormaterial
		  :create :name :grey36
		  :diffuse #f(0.359375 0.359375 0.359375))
	(instance colormaterial
		  :create :name :gray37
		  :diffuse #f(0.3671875 0.3671875 0.3671875))
	(instance colormaterial
		  :create :name :grey37
		  :diffuse #f(0.3671875 0.3671875 0.3671875))
	(instance colormaterial
		  :create :name :gray38
		  :diffuse #f(0.37890625 0.37890625 0.37890625))
	(instance colormaterial
		  :create :name :grey38
		  :diffuse #f(0.37890625 0.37890625 0.37890625))
	(instance colormaterial
		  :create :name :gray39
		  :diffuse #f(0.38671875 0.38671875 0.38671875))
	(instance colormaterial
		  :create :name :grey39
		  :diffuse #f(0.38671875 0.38671875 0.38671875))
	(instance colormaterial
		  :create :name :gray40
		  :diffuse #f(0.3984375 0.3984375 0.3984375))
	(instance colormaterial
		  :create :name :grey40
		  :diffuse #f(0.3984375 0.3984375 0.3984375))
	(instance colormaterial
		  :create :name :gray41
		  :diffuse #f(0.41015625 0.41015625 0.41015625))
	(instance colormaterial
		  :create :name :grey41
		  :diffuse #f(0.41015625 0.41015625 0.41015625))
	(instance colormaterial
		  :create :name :gray42
		  :diffuse #f(0.41796875 0.41796875 0.41796875))
	(instance colormaterial
		  :create :name :grey42
		  :diffuse #f(0.41796875 0.41796875 0.41796875))
	(instance colormaterial
		  :create :name :gray43
		  :diffuse #f(0.4296875 0.4296875 0.4296875))
	(instance colormaterial
		  :create :name :grey43
		  :diffuse #f(0.4296875 0.4296875 0.4296875))
	(instance colormaterial
		  :create :name :gray44
		  :diffuse #f(0.4375 0.4375 0.4375))
	(instance colormaterial
		  :create :name :grey44
		  :diffuse #f(0.4375 0.4375 0.4375))
	(instance colormaterial
		  :create :name :gray45
		  :diffuse #f(0.44921875 0.44921875 0.44921875))
	(instance colormaterial
		  :create :name :grey45
		  :diffuse #f(0.44921875 0.44921875 0.44921875))
	(instance colormaterial
		  :create :name :gray46
		  :diffuse #f(0.45703125 0.45703125 0.45703125))
	(instance colormaterial
		  :create :name :grey46
		  :diffuse #f(0.45703125 0.45703125 0.45703125))
	(instance colormaterial
		  :create :name :gray47
		  :diffuse #f(0.46875 0.46875 0.46875))
	(instance colormaterial
		  :create :name :grey47
		  :diffuse #f(0.46875 0.46875 0.46875))
	(instance colormaterial
		  :create :name :gray48
		  :diffuse #f(0.4765625 0.4765625 0.4765625))
	(instance colormaterial
		  :create :name :grey48
		  :diffuse #f(0.4765625 0.4765625 0.4765625))
	(instance colormaterial
		  :create :name :gray49
		  :diffuse #f(0.48828125 0.48828125 0.48828125))
	(instance colormaterial
		  :create :name :grey49
		  :diffuse #f(0.48828125 0.48828125 0.48828125))
	(instance colormaterial
		  :create :name :gray50
		  :diffuse #f(0.49609375 0.49609375 0.49609375))
	(instance colormaterial
		  :create :name :grey50
		  :diffuse #f(0.49609375 0.49609375 0.49609375))
	(instance colormaterial
		  :create :name :gray51
		  :diffuse #f(0.5078125 0.5078125 0.5078125))
	(instance colormaterial
		  :create :name :grey51
		  :diffuse #f(0.5078125 0.5078125 0.5078125))
	(instance colormaterial
		  :create :name :gray52
		  :diffuse #f(0.51953125 0.51953125 0.51953125))
	(instance colormaterial
		  :create :name :grey52
		  :diffuse #f(0.51953125 0.51953125 0.51953125))
	(instance colormaterial
		  :create :name :gray53
		  :diffuse #f(0.52734375 0.52734375 0.52734375))
	(instance colormaterial
		  :create :name :grey53
		  :diffuse #f(0.52734375 0.52734375 0.52734375))
	(instance colormaterial
		  :create :name :gray54
		  :diffuse #f(0.5390625 0.5390625 0.5390625))
	(instance colormaterial
		  :create :name :grey54
		  :diffuse #f(0.5390625 0.5390625 0.5390625))
	(instance colormaterial
		  :create :name :gray55
		  :diffuse #f(0.546875 0.546875 0.546875))
	(instance colormaterial
		  :create :name :grey55
		  :diffuse #f(0.546875 0.546875 0.546875))
	(instance colormaterial
		  :create :name :gray56
		  :diffuse #f(0.55859375 0.55859375 0.55859375))
	(instance colormaterial
		  :create :name :grey56
		  :diffuse #f(0.55859375 0.55859375 0.55859375))
	(instance colormaterial
		  :create :name :gray57
		  :diffuse #f(0.56640625 0.56640625 0.56640625))
	(instance colormaterial
		  :create :name :grey57
		  :diffuse #f(0.56640625 0.56640625 0.56640625))
	(instance colormaterial
		  :create :name :gray58
		  :diffuse #f(0.578125 0.578125 0.578125))
	(instance colormaterial
		  :create :name :grey58
		  :diffuse #f(0.578125 0.578125 0.578125))
	(instance colormaterial
		  :create :name :gray59
		  :diffuse #f(0.5859375 0.5859375 0.5859375))
	(instance colormaterial
		  :create :name :grey59
		  :diffuse #f(0.5859375 0.5859375 0.5859375))
	(instance colormaterial
		  :create :name :gray60
		  :diffuse #f(0.59765625 0.59765625 0.59765625))
	(instance colormaterial
		  :create :name :grey60
		  :diffuse #f(0.59765625 0.59765625 0.59765625))
	(instance colormaterial
		  :create :name :gray61
		  :diffuse #f(0.609375 0.609375 0.609375))
	(instance colormaterial
		  :create :name :grey61
		  :diffuse #f(0.609375 0.609375 0.609375))
	(instance colormaterial
		  :create :name :gray62
		  :diffuse #f(0.6171875 0.6171875 0.6171875))
	(instance colormaterial
		  :create :name :grey62
		  :diffuse #f(0.6171875 0.6171875 0.6171875))
	(instance colormaterial
		  :create :name :gray63
		  :diffuse #f(0.62890625 0.62890625 0.62890625))
	(instance colormaterial
		  :create :name :grey63
		  :diffuse #f(0.62890625 0.62890625 0.62890625))
	(instance colormaterial
		  :create :name :gray64
		  :diffuse #f(0.63671875 0.63671875 0.63671875))
	(instance colormaterial
		  :create :name :grey64
		  :diffuse #f(0.63671875 0.63671875 0.63671875))
	(instance colormaterial
		  :create :name :gray65
		  :diffuse #f(0.6484375 0.6484375 0.6484375))
	(instance colormaterial
		  :create :name :grey65
		  :diffuse #f(0.6484375 0.6484375 0.6484375))
	(instance colormaterial
		  :create :name :gray66
		  :diffuse #f(0.65625 0.65625 0.65625))
	(instance colormaterial
		  :create :name :grey66
		  :diffuse #f(0.65625 0.65625 0.65625))
	(instance colormaterial
		  :create :name :gray67
		  :diffuse #f(0.66796875 0.66796875 0.66796875))
	(instance colormaterial
		  :create :name :grey67
		  :diffuse #f(0.66796875 0.66796875 0.66796875))
	(instance colormaterial
		  :create :name :gray68
		  :diffuse #f(0.67578125 0.67578125 0.67578125))
	(instance colormaterial
		  :create :name :grey68
		  :diffuse #f(0.67578125 0.67578125 0.67578125))
	(instance colormaterial
		  :create :name :gray69
		  :diffuse #f(0.6875 0.6875 0.6875))
	(instance colormaterial
		  :create :name :grey69
		  :diffuse #f(0.6875 0.6875 0.6875))
	(instance colormaterial
		  :create :name :gray70
		  :diffuse #f(0.69921875 0.69921875 0.69921875))
	(instance colormaterial
		  :create :name :grey70
		  :diffuse #f(0.69921875 0.69921875 0.69921875))
	(instance colormaterial
		  :create :name :gray71
		  :diffuse #f(0.70703125 0.70703125 0.70703125))
	(instance colormaterial
		  :create :name :grey71
		  :diffuse #f(0.70703125 0.70703125 0.70703125))
	(instance colormaterial
		  :create :name :gray72
		  :diffuse #f(0.71875 0.71875 0.71875))
	(instance colormaterial
		  :create :name :grey72
		  :diffuse #f(0.71875 0.71875 0.71875))
	(instance colormaterial
		  :create :name :gray73
		  :diffuse #f(0.7265625 0.7265625 0.7265625))
	(instance colormaterial
		  :create :name :grey73
		  :diffuse #f(0.7265625 0.7265625 0.7265625))
	(instance colormaterial
		  :create :name :gray74
		  :diffuse #f(0.73828125 0.73828125 0.73828125))
	(instance colormaterial
		  :create :name :grey74
		  :diffuse #f(0.73828125 0.73828125 0.73828125))
	(instance colormaterial
		  :create :name :gray75
		  :diffuse #f(0.74609375 0.74609375 0.74609375))
	(instance colormaterial
		  :create :name :grey75
		  :diffuse #f(0.74609375 0.74609375 0.74609375))
	(instance colormaterial
		  :create :name :gray76
		  :diffuse #f(0.7578125 0.7578125 0.7578125))
	(instance colormaterial
		  :create :name :grey76
		  :diffuse #f(0.7578125 0.7578125 0.7578125))
	(instance colormaterial
		  :create :name :gray77
		  :diffuse #f(0.765625 0.765625 0.765625))
	(instance colormaterial
		  :create :name :grey77
		  :diffuse #f(0.765625 0.765625 0.765625))
	(instance colormaterial
		  :create :name :gray78
		  :diffuse #f(0.77734375 0.77734375 0.77734375))
	(instance colormaterial
		  :create :name :grey78
		  :diffuse #f(0.77734375 0.77734375 0.77734375))
	(instance colormaterial
		  :create :name :gray79
		  :diffuse #f(0.78515625 0.78515625 0.78515625))
	(instance colormaterial
		  :create :name :grey79
		  :diffuse #f(0.78515625 0.78515625 0.78515625))
	(instance colormaterial
		  :create :name :gray80
		  :diffuse #f(0.796875 0.796875 0.796875))
	(instance colormaterial
		  :create :name :grey80
		  :diffuse #f(0.796875 0.796875 0.796875))
	(instance colormaterial
		  :create :name :gray81
		  :diffuse #f(0.80859375 0.80859375 0.80859375))
	(instance colormaterial
		  :create :name :grey81
		  :diffuse #f(0.80859375 0.80859375 0.80859375))
	(instance colormaterial
		  :create :name :gray82
		  :diffuse #f(0.81640625 0.81640625 0.81640625))
	(instance colormaterial
		  :create :name :grey82
		  :diffuse #f(0.81640625 0.81640625 0.81640625))
	(instance colormaterial
		  :create :name :gray83
		  :diffuse #f(0.828125 0.828125 0.828125))
	(instance colormaterial
		  :create :name :grey83
		  :diffuse #f(0.828125 0.828125 0.828125))
	(instance colormaterial
		  :create :name :gray84
		  :diffuse #f(0.8359375 0.8359375 0.8359375))
	(instance colormaterial
		  :create :name :grey84
		  :diffuse #f(0.8359375 0.8359375 0.8359375))
	(instance colormaterial
		  :create :name :gray85
		  :diffuse #f(0.84765625 0.84765625 0.84765625))
	(instance colormaterial
		  :create :name :grey85
		  :diffuse #f(0.84765625 0.84765625 0.84765625))
	(instance colormaterial
		  :create :name :gray86
		  :diffuse #f(0.85546875 0.85546875 0.85546875))
	(instance colormaterial
		  :create :name :grey86
		  :diffuse #f(0.85546875 0.85546875 0.85546875))
	(instance colormaterial
		  :create :name :gray87
		  :diffuse #f(0.8671875 0.8671875 0.8671875))
	(instance colormaterial
		  :create :name :grey87
		  :diffuse #f(0.8671875 0.8671875 0.8671875))
	(instance colormaterial
		  :create :name :gray88
		  :diffuse #f(0.875 0.875 0.875))
	(instance colormaterial
		  :create :name :grey88
		  :diffuse #f(0.875 0.875 0.875))
	(instance colormaterial
		  :create :name :gray89
		  :diffuse #f(0.88671875 0.88671875 0.88671875))
	(instance colormaterial
		  :create :name :grey89
		  :diffuse #f(0.88671875 0.88671875 0.88671875))
	(instance colormaterial
		  :create :name :gray90
		  :diffuse #f(0.89453125 0.89453125 0.89453125))
	(instance colormaterial
		  :create :name :grey90
		  :diffuse #f(0.89453125 0.89453125 0.89453125))
	(instance colormaterial
		  :create :name :gray91
		  :diffuse #f(0.90625 0.90625 0.90625))
	(instance colormaterial
		  :create :name :grey91
		  :diffuse #f(0.90625 0.90625 0.90625))
	(instance colormaterial
		  :create :name :gray92
		  :diffuse #f(0.91796875 0.91796875 0.91796875))
	(instance colormaterial
		  :create :name :grey92
		  :diffuse #f(0.91796875 0.91796875 0.91796875))
	(instance colormaterial
		  :create :name :gray93
		  :diffuse #f(0.92578125 0.92578125 0.92578125))
	(instance colormaterial
		  :create :name :grey93
		  :diffuse #f(0.92578125 0.92578125 0.92578125))
	(instance colormaterial
		  :create :name :gray94
		  :diffuse #f(0.9375 0.9375 0.9375))
	(instance colormaterial
		  :create :name :grey94
		  :diffuse #f(0.9375 0.9375 0.9375))
	(instance colormaterial
		  :create :name :gray95
		  :diffuse #f(0.9453125 0.9453125 0.9453125))
	(instance colormaterial
		  :create :name :grey95
		  :diffuse #f(0.9453125 0.9453125 0.9453125))
	(instance colormaterial
		  :create :name :gray96
		  :diffuse #f(0.95703125 0.95703125 0.95703125))
	(instance colormaterial
		  :create :name :grey96
		  :diffuse #f(0.95703125 0.95703125 0.95703125))
	(instance colormaterial
		  :create :name :gray97
		  :diffuse #f(0.96484375 0.96484375 0.96484375))
	(instance colormaterial
		  :create :name :grey97
		  :diffuse #f(0.96484375 0.96484375 0.96484375))
	(instance colormaterial
		  :create :name :gray98
		  :diffuse #f(0.9765625 0.9765625 0.9765625))
	(instance colormaterial
		  :create :name :grey98
		  :diffuse #f(0.9765625 0.9765625 0.9765625))
	(instance colormaterial
		  :create :name :gray99
		  :diffuse #f(0.984375 0.984375 0.984375))
	(instance colormaterial
		  :create :name :grey99
		  :diffuse #f(0.984375 0.984375 0.984375))
	(instance colormaterial
		  :create :name :gray100
		  :diffuse #f(0.99609375 0.99609375 0.99609375))
	(instance colormaterial
		  :create :name :grey100
		  :diffuse #f(0.99609375 0.99609375 0.99609375))
	(instance colormaterial
		  :create :name :DarkGrey
		  :diffuse #f(0.66015625 0.66015625 0.66015625))
	(instance colormaterial
		  :create :name :DarkGray
		  :diffuse #f(0.66015625 0.66015625 0.66015625))
	(instance colormaterial
		  :create :name :DarkBlue
		  :diffuse #f(0 0 0.54296875))
	(instance colormaterial
		  :create :name :DarkCyan
		  :diffuse #f(0 0.54296875 0.54296875))
	(instance colormaterial
		  :create :name :DarkMagenta
		  :diffuse #f(0.54296875 0 0.54296875))
	(instance colormaterial
		  :create :name :DarkRed
		  :diffuse #f(0.54296875 0 0))
	(instance colormaterial
		  :create :name :LightGreen
		  :diffuse #f(0.5625 0.9296875 0.5625))
	;;
	;;  added from rosk's sensor-colors.l
	;;
	(instance colormaterial
		  :create :name :intensity-0
		  :diffuse #f(0 0 0 ))
	(instance colormaterial
		  :create :name :intensity-1
		  :diffuse #f(0.00392157 0.00392157 0.00392157 ))
	(instance colormaterial
		  :create :name :intensity-2
		  :diffuse #f(0.00784314 0.00784314 0.00784314 ))
	(instance colormaterial
		  :create :name :intensity-3
		  :diffuse #f(0.0117647 0.0117647 0.0117647 ))
	(instance colormaterial
		  :create :name :intensity-4
		  :diffuse #f(0.0156863 0.0156863 0.0156863 ))
	(instance colormaterial
		  :create :name :intensity-5
		  :diffuse #f(0.0196078 0.0196078 0.0196078 ))
	(instance colormaterial
		  :create :name :intensity-6
		  :diffuse #f(0.0235294 0.0235294 0.0235294 ))
	(instance colormaterial
		  :create :name :intensity-7
		  :diffuse #f(0.027451 0.027451 0.027451 ))
	(instance colormaterial
		  :create :name :intensity-8
		  :diffuse #f(0.0313725 0.0313725 0.0313725 ))
	(instance colormaterial
		  :create :name :intensity-9
		  :diffuse #f(0.0352941 0.0352941 0.0352941 ))
	(instance colormaterial
		  :create :name :intensity-10
		  :diffuse #f(0.0392157 0.0392157 0.0392157 ))
	(instance colormaterial
		  :create :name :intensity-11
		  :diffuse #f(0.0431373 0.0431373 0.0431373 ))
	(instance colormaterial
		  :create :name :intensity-12
		  :diffuse #f(0.0470588 0.0470588 0.0470588 ))
	(instance colormaterial
		  :create :name :intensity-13
		  :diffuse #f(0.0509804 0.0509804 0.0509804 ))
	(instance colormaterial
		  :create :name :intensity-14
		  :diffuse #f(0.054902 0.054902 0.054902 ))
	(instance colormaterial
		  :create :name :intensity-15
		  :diffuse #f(0.0588235 0.0588235 0.0588235 ))
	(instance colormaterial
		  :create :name :intensity-16
		  :diffuse #f(0.0627451 0.0627451 0.0627451 ))
	(instance colormaterial
		  :create :name :intensity-17
		  :diffuse #f(0.0666667 0.0666667 0.0666667 ))
	(instance colormaterial
		  :create :name :intensity-18
		  :diffuse #f(0.0705882 0.0705882 0.0705882 ))
	(instance colormaterial
		  :create :name :intensity-19
		  :diffuse #f(0.0745098 0.0745098 0.0745098 ))
	(instance colormaterial
		  :create :name :intensity-20
		  :diffuse #f(0.0784314 0.0784314 0.0784314 ))
	(instance colormaterial
		  :create :name :intensity-21
		  :diffuse #f(0.0823529 0.0823529 0.0823529 ))
	(instance colormaterial
		  :create :name :intensity-22
		  :diffuse #f(0.0862745 0.0862745 0.0862745 ))
	(instance colormaterial
		  :create :name :intensity-23
		  :diffuse #f(0.0901961 0.0901961 0.0901961 ))
	(instance colormaterial
		  :create :name :intensity-24
		  :diffuse #f(0.0941176 0.0941176 0.0941176 ))
	(instance colormaterial
		  :create :name :intensity-25
		  :diffuse #f(0.0980392 0.0980392 0.0980392 ))
	(instance colormaterial
		  :create :name :intensity-26
		  :diffuse #f(0.101961 0.101961 0.101961 ))
	(instance colormaterial
		  :create :name :intensity-27
		  :diffuse #f(0.105882 0.105882 0.105882 ))
	(instance colormaterial
		  :create :name :intensity-28
		  :diffuse #f(0.109804 0.109804 0.109804 ))
	(instance colormaterial
		  :create :name :intensity-29
		  :diffuse #f(0.113725 0.113725 0.113725 ))
	(instance colormaterial
		  :create :name :intensity-30
		  :diffuse #f(0.117647 0.117647 0.117647 ))
	(instance colormaterial
		  :create :name :intensity-31
		  :diffuse #f(0.121569 0.121569 0.121569 ))
	(instance colormaterial
		  :create :name :intensity-32
		  :diffuse #f(0.12549 0.12549 0.12549 ))
	(instance colormaterial
		  :create :name :intensity-33
		  :diffuse #f(0.129412 0.129412 0.129412 ))
	(instance colormaterial
		  :create :name :intensity-34
		  :diffuse #f(0.133333 0.133333 0.133333 ))
	(instance colormaterial
		  :create :name :intensity-35
		  :diffuse #f(0.137255 0.137255 0.137255 ))
	(instance colormaterial
		  :create :name :intensity-36
		  :diffuse #f(0.141176 0.141176 0.141176 ))
	(instance colormaterial
		  :create :name :intensity-37
		  :diffuse #f(0.145098 0.145098 0.145098 ))
	(instance colormaterial
		  :create :name :intensity-38
		  :diffuse #f(0.14902 0.14902 0.14902 ))
	(instance colormaterial
		  :create :name :intensity-39
		  :diffuse #f(0.152941 0.152941 0.152941 ))
	(instance colormaterial
		  :create :name :intensity-40
		  :diffuse #f(0.156863 0.156863 0.156863 ))
	(instance colormaterial
		  :create :name :intensity-41
		  :diffuse #f(0.160784 0.160784 0.160784 ))
	(instance colormaterial
		  :create :name :intensity-42
		  :diffuse #f(0.164706 0.164706 0.164706 ))
	(instance colormaterial
		  :create :name :intensity-43
		  :diffuse #f(0.168627 0.168627 0.168627 ))
	(instance colormaterial
		  :create :name :intensity-44
		  :diffuse #f(0.172549 0.172549 0.172549 ))
	(instance colormaterial
		  :create :name :intensity-45
		  :diffuse #f(0.176471 0.176471 0.176471 ))
	(instance colormaterial
		  :create :name :intensity-46
		  :diffuse #f(0.180392 0.180392 0.180392 ))
	(instance colormaterial
		  :create :name :intensity-47
		  :diffuse #f(0.184314 0.184314 0.184314 ))
	(instance colormaterial
		  :create :name :intensity-48
		  :diffuse #f(0.188235 0.188235 0.188235 ))
	(instance colormaterial
		  :create :name :intensity-49
		  :diffuse #f(0.192157 0.192157 0.192157 ))
	(instance colormaterial
		  :create :name :intensity-50
		  :diffuse #f(0.196078 0.196078 0.196078 ))
	(instance colormaterial
		  :create :name :intensity-51
		  :diffuse #f(0.2 0.2 0.2 ))
	(instance colormaterial
		  :create :name :intensity-52
		  :diffuse #f(0.203922 0.203922 0.203922 ))
	(instance colormaterial
		  :create :name :intensity-53
		  :diffuse #f(0.207843 0.207843 0.207843 ))
	(instance colormaterial
		  :create :name :intensity-54
		  :diffuse #f(0.211765 0.211765 0.211765 ))
	(instance colormaterial
		  :create :name :intensity-55
		  :diffuse #f(0.215686 0.215686 0.215686 ))
	(instance colormaterial
		  :create :name :intensity-56
		  :diffuse #f(0.219608 0.219608 0.219608 ))
	(instance colormaterial
		  :create :name :intensity-57
		  :diffuse #f(0.223529 0.223529 0.223529 ))
	(instance colormaterial
		  :create :name :intensity-58
		  :diffuse #f(0.227451 0.227451 0.227451 ))
	(instance colormaterial
		  :create :name :intensity-59
		  :diffuse #f(0.231373 0.231373 0.231373 ))
	(instance colormaterial
		  :create :name :intensity-60
		  :diffuse #f(0.235294 0.235294 0.235294 ))
	(instance colormaterial
		  :create :name :intensity-61
		  :diffuse #f(0.239216 0.239216 0.239216 ))
	(instance colormaterial
		  :create :name :intensity-62
		  :diffuse #f(0.243137 0.243137 0.243137 ))
	(instance colormaterial
		  :create :name :intensity-63
		  :diffuse #f(0.247059 0.247059 0.247059 ))
	(instance colormaterial
		  :create :name :intensity-64
		  :diffuse #f(0.25098 0.25098 0.25098 ))
	(instance colormaterial
		  :create :name :intensity-65
		  :diffuse #f(0.254902 0.254902 0.254902 ))
	(instance colormaterial
		  :create :name :intensity-66
		  :diffuse #f(0.258824 0.258824 0.258824 ))
	(instance colormaterial
		  :create :name :intensity-67
		  :diffuse #f(0.262745 0.262745 0.262745 ))
	(instance colormaterial
		  :create :name :intensity-68
		  :diffuse #f(0.266667 0.266667 0.266667 ))
	(instance colormaterial
		  :create :name :intensity-69
		  :diffuse #f(0.270588 0.270588 0.270588 ))
	(instance colormaterial
		  :create :name :intensity-70
		  :diffuse #f(0.27451 0.27451 0.27451 ))
	(instance colormaterial
		  :create :name :intensity-71
		  :diffuse #f(0.278431 0.278431 0.278431 ))
	(instance colormaterial
		  :create :name :intensity-72
		  :diffuse #f(0.282353 0.282353 0.282353 ))
	(instance colormaterial
		  :create :name :intensity-73
		  :diffuse #f(0.286275 0.286275 0.286275 ))
	(instance colormaterial
		  :create :name :intensity-74
		  :diffuse #f(0.290196 0.290196 0.290196 ))
	(instance colormaterial
		  :create :name :intensity-75
		  :diffuse #f(0.294118 0.294118 0.294118 ))
	(instance colormaterial
		  :create :name :intensity-76
		  :diffuse #f(0.298039 0.298039 0.298039 ))
	(instance colormaterial
		  :create :name :intensity-77
		  :diffuse #f(0.301961 0.301961 0.301961 ))
	(instance colormaterial
		  :create :name :intensity-78
		  :diffuse #f(0.305882 0.305882 0.305882 ))
	(instance colormaterial
		  :create :name :intensity-79
		  :diffuse #f(0.309804 0.309804 0.309804 ))
	(instance colormaterial
		  :create :name :intensity-80
		  :diffuse #f(0.313725 0.313725 0.313725 ))
	(instance colormaterial
		  :create :name :intensity-81
		  :diffuse #f(0.317647 0.317647 0.317647 ))
	(instance colormaterial
		  :create :name :intensity-82
		  :diffuse #f(0.321569 0.321569 0.321569 ))
	(instance colormaterial
		  :create :name :intensity-83
		  :diffuse #f(0.32549 0.32549 0.32549 ))
	(instance colormaterial
		  :create :name :intensity-84
		  :diffuse #f(0.329412 0.329412 0.329412 ))
	(instance colormaterial
		  :create :name :intensity-85
		  :diffuse #f(0.333333 0.333333 0.333333 ))
	(instance colormaterial
		  :create :name :intensity-86
		  :diffuse #f(0.337255 0.337255 0.337255 ))
	(instance colormaterial
		  :create :name :intensity-87
		  :diffuse #f(0.341176 0.341176 0.341176 ))
	(instance colormaterial
		  :create :name :intensity-88
		  :diffuse #f(0.345098 0.345098 0.345098 ))
	(instance colormaterial
		  :create :name :intensity-89
		  :diffuse #f(0.34902 0.34902 0.34902 ))
	(instance colormaterial
		  :create :name :intensity-90
		  :diffuse #f(0.352941 0.352941 0.352941 ))
	(instance colormaterial
		  :create :name :intensity-91
		  :diffuse #f(0.356863 0.356863 0.356863 ))
	(instance colormaterial
		  :create :name :intensity-92
		  :diffuse #f(0.360784 0.360784 0.360784 ))
	(instance colormaterial
		  :create :name :intensity-93
		  :diffuse #f(0.364706 0.364706 0.364706 ))
	(instance colormaterial
		  :create :name :intensity-94
		  :diffuse #f(0.368627 0.368627 0.368627 ))
	(instance colormaterial
		  :create :name :intensity-95
		  :diffuse #f(0.372549 0.372549 0.372549 ))
	(instance colormaterial
		  :create :name :intensity-96
		  :diffuse #f(0.376471 0.376471 0.376471 ))
	(instance colormaterial
		  :create :name :intensity-97
		  :diffuse #f(0.380392 0.380392 0.380392 ))
	(instance colormaterial
		  :create :name :intensity-98
		  :diffuse #f(0.384314 0.384314 0.384314 ))
	(instance colormaterial
		  :create :name :intensity-99
		  :diffuse #f(0.388235 0.388235 0.388235 ))
	(instance colormaterial
		  :create :name :intensity-100
		  :diffuse #f(0.392157 0.392157 0.392157 ))
	(instance colormaterial
		  :create :name :intensity-101
		  :diffuse #f(0.396078 0.396078 0.396078 ))
	(instance colormaterial
		  :create :name :intensity-102
		  :diffuse #f(0.4 0.4 0.4 ))
	(instance colormaterial
		  :create :name :intensity-103
		  :diffuse #f(0.403922 0.403922 0.403922 ))
	(instance colormaterial
		  :create :name :intensity-104
		  :diffuse #f(0.407843 0.407843 0.407843 ))
	(instance colormaterial
		  :create :name :intensity-105
		  :diffuse #f(0.411765 0.411765 0.411765 ))
	(instance colormaterial
		  :create :name :intensity-106
		  :diffuse #f(0.415686 0.415686 0.415686 ))
	(instance colormaterial
		  :create :name :intensity-107
		  :diffuse #f(0.419608 0.419608 0.419608 ))
	(instance colormaterial
		  :create :name :intensity-108
		  :diffuse #f(0.423529 0.423529 0.423529 ))
	(instance colormaterial
		  :create :name :intensity-109
		  :diffuse #f(0.427451 0.427451 0.427451 ))
	(instance colormaterial
		  :create :name :intensity-110
		  :diffuse #f(0.431373 0.431373 0.431373 ))
	(instance colormaterial
		  :create :name :intensity-111
		  :diffuse #f(0.435294 0.435294 0.435294 ))
	(instance colormaterial
		  :create :name :intensity-112
		  :diffuse #f(0.439216 0.439216 0.439216 ))
	(instance colormaterial
		  :create :name :intensity-113
		  :diffuse #f(0.443137 0.443137 0.443137 ))
	(instance colormaterial
		  :create :name :intensity-114
		  :diffuse #f(0.447059 0.447059 0.447059 ))
	(instance colormaterial
		  :create :name :intensity-115
		  :diffuse #f(0.45098 0.45098 0.45098 ))
	(instance colormaterial
		  :create :name :intensity-116
		  :diffuse #f(0.454902 0.454902 0.454902 ))
	(instance colormaterial
		  :create :name :intensity-117
		  :diffuse #f(0.458824 0.458824 0.458824 ))
	(instance colormaterial
		  :create :name :intensity-118
		  :diffuse #f(0.462745 0.462745 0.462745 ))
	(instance colormaterial
		  :create :name :intensity-119
		  :diffuse #f(0.466667 0.466667 0.466667 ))
	(instance colormaterial
		  :create :name :intensity-120
		  :diffuse #f(0.470588 0.470588 0.470588 ))
	(instance colormaterial
		  :create :name :intensity-121
		  :diffuse #f(0.47451 0.47451 0.47451 ))
	(instance colormaterial
		  :create :name :intensity-122
		  :diffuse #f(0.478431 0.478431 0.478431 ))
	(instance colormaterial
		  :create :name :intensity-123
		  :diffuse #f(0.482353 0.482353 0.482353 ))
	(instance colormaterial
		  :create :name :intensity-124
		  :diffuse #f(0.486275 0.486275 0.486275 ))
	(instance colormaterial
		  :create :name :intensity-125
		  :diffuse #f(0.490196 0.490196 0.490196 ))
	(instance colormaterial
		  :create :name :intensity-126
		  :diffuse #f(0.494118 0.494118 0.494118 ))
	(instance colormaterial
		  :create :name :intensity-127
		  :diffuse #f(0.498039 0.498039 0.498039 ))
	(instance colormaterial
		  :create :name :intensity-128
		  :diffuse #f(0.501961 0.501961 0.501961 ))
	(instance colormaterial
		  :create :name :intensity-129
		  :diffuse #f(0.505882 0.505882 0.505882 ))
	(instance colormaterial
		  :create :name :intensity-130
		  :diffuse #f(0.509804 0.509804 0.509804 ))
	(instance colormaterial
		  :create :name :intensity-131
		  :diffuse #f(0.513725 0.513725 0.513725 ))
	(instance colormaterial
		  :create :name :intensity-132
		  :diffuse #f(0.517647 0.517647 0.517647 ))
	(instance colormaterial
		  :create :name :intensity-133
		  :diffuse #f(0.521569 0.521569 0.521569 ))
	(instance colormaterial
		  :create :name :intensity-134
		  :diffuse #f(0.52549 0.52549 0.52549 ))
	(instance colormaterial
		  :create :name :intensity-135
		  :diffuse #f(0.529412 0.529412 0.529412 ))
	(instance colormaterial
		  :create :name :intensity-136
		  :diffuse #f(0.533333 0.533333 0.533333 ))
	(instance colormaterial
		  :create :name :intensity-137
		  :diffuse #f(0.537255 0.537255 0.537255 ))
	(instance colormaterial
		  :create :name :intensity-138
		  :diffuse #f(0.541176 0.541176 0.541176 ))
	(instance colormaterial
		  :create :name :intensity-139
		  :diffuse #f(0.545098 0.545098 0.545098 ))
	(instance colormaterial
		  :create :name :intensity-140
		  :diffuse #f(0.54902 0.54902 0.54902 ))
	(instance colormaterial
		  :create :name :intensity-141
		  :diffuse #f(0.552941 0.552941 0.552941 ))
	(instance colormaterial
		  :create :name :intensity-142
		  :diffuse #f(0.556863 0.556863 0.556863 ))
	(instance colormaterial
		  :create :name :intensity-143
		  :diffuse #f(0.560784 0.560784 0.560784 ))
	(instance colormaterial
		  :create :name :intensity-144
		  :diffuse #f(0.564706 0.564706 0.564706 ))
	(instance colormaterial
		  :create :name :intensity-145
		  :diffuse #f(0.568627 0.568627 0.568627 ))
	(instance colormaterial
		  :create :name :intensity-146
		  :diffuse #f(0.572549 0.572549 0.572549 ))
	(instance colormaterial
		  :create :name :intensity-147
		  :diffuse #f(0.576471 0.576471 0.576471 ))
	(instance colormaterial
		  :create :name :intensity-148
		  :diffuse #f(0.580392 0.580392 0.580392 ))
	(instance colormaterial
		  :create :name :intensity-149
		  :diffuse #f(0.584314 0.584314 0.584314 ))
	(instance colormaterial
		  :create :name :intensity-150
		  :diffuse #f(0.588235 0.588235 0.588235 ))
	(instance colormaterial
		  :create :name :intensity-151
		  :diffuse #f(0.592157 0.592157 0.592157 ))
	(instance colormaterial
		  :create :name :intensity-152
		  :diffuse #f(0.596078 0.596078 0.596078 ))
	(instance colormaterial
		  :create :name :intensity-153
		  :diffuse #f(0.6 0.6 0.6 ))
	(instance colormaterial
		  :create :name :intensity-154
		  :diffuse #f(0.603922 0.603922 0.603922 ))
	(instance colormaterial
		  :create :name :intensity-155
		  :diffuse #f(0.607843 0.607843 0.607843 ))
	(instance colormaterial
		  :create :name :intensity-156
		  :diffuse #f(0.611765 0.611765 0.611765 ))
	(instance colormaterial
		  :create :name :intensity-157
		  :diffuse #f(0.615686 0.615686 0.615686 ))
	(instance colormaterial
		  :create :name :intensity-158
		  :diffuse #f(0.619608 0.619608 0.619608 ))
	(instance colormaterial
		  :create :name :intensity-159
		  :diffuse #f(0.623529 0.623529 0.623529 ))
	(instance colormaterial
		  :create :name :intensity-160
		  :diffuse #f(0.627451 0.627451 0.627451 ))
	(instance colormaterial
		  :create :name :intensity-161
		  :diffuse #f(0.631373 0.631373 0.631373 ))
	(instance colormaterial
		  :create :name :intensity-162
		  :diffuse #f(0.635294 0.635294 0.635294 ))
	(instance colormaterial
		  :create :name :intensity-163
		  :diffuse #f(0.639216 0.639216 0.639216 ))
	(instance colormaterial
		  :create :name :intensity-164
		  :diffuse #f(0.643137 0.643137 0.643137 ))
	(instance colormaterial
		  :create :name :intensity-165
		  :diffuse #f(0.647059 0.647059 0.647059 ))
	(instance colormaterial
		  :create :name :intensity-166
		  :diffuse #f(0.65098 0.65098 0.65098 ))
	(instance colormaterial
		  :create :name :intensity-167
		  :diffuse #f(0.654902 0.654902 0.654902 ))
	(instance colormaterial
		  :create :name :intensity-168
		  :diffuse #f(0.658824 0.658824 0.658824 ))
	(instance colormaterial
		  :create :name :intensity-169
		  :diffuse #f(0.662745 0.662745 0.662745 ))
	(instance colormaterial
		  :create :name :intensity-170
		  :diffuse #f(0.666667 0.666667 0.666667 ))
	(instance colormaterial
		  :create :name :intensity-171
		  :diffuse #f(0.670588 0.670588 0.670588 ))
	(instance colormaterial
		  :create :name :intensity-172
		  :diffuse #f(0.67451 0.67451 0.67451 ))
	(instance colormaterial
		  :create :name :intensity-173
		  :diffuse #f(0.678431 0.678431 0.678431 ))
	(instance colormaterial
		  :create :name :intensity-174
		  :diffuse #f(0.682353 0.682353 0.682353 ))
	(instance colormaterial
		  :create :name :intensity-175
		  :diffuse #f(0.686275 0.686275 0.686275 ))
	(instance colormaterial
		  :create :name :intensity-176
		  :diffuse #f(0.690196 0.690196 0.690196 ))
	(instance colormaterial
		  :create :name :intensity-177
		  :diffuse #f(0.694118 0.694118 0.694118 ))
	(instance colormaterial
		  :create :name :intensity-178
		  :diffuse #f(0.698039 0.698039 0.698039 ))
	(instance colormaterial
		  :create :name :intensity-179
		  :diffuse #f(0.701961 0.701961 0.701961 ))
	(instance colormaterial
		  :create :name :intensity-180
		  :diffuse #f(0.705882 0.705882 0.705882 ))
	(instance colormaterial
		  :create :name :intensity-181
		  :diffuse #f(0.709804 0.709804 0.709804 ))
	(instance colormaterial
		  :create :name :intensity-182
		  :diffuse #f(0.713725 0.713725 0.713725 ))
	(instance colormaterial
		  :create :name :intensity-183
		  :diffuse #f(0.717647 0.717647 0.717647 ))
	(instance colormaterial
		  :create :name :intensity-184
		  :diffuse #f(0.721569 0.721569 0.721569 ))
	(instance colormaterial
		  :create :name :intensity-185
		  :diffuse #f(0.72549 0.72549 0.72549 ))
	(instance colormaterial
		  :create :name :intensity-186
		  :diffuse #f(0.729412 0.729412 0.729412 ))
	(instance colormaterial
		  :create :name :intensity-187
		  :diffuse #f(0.733333 0.733333 0.733333 ))
	(instance colormaterial
		  :create :name :intensity-188
		  :diffuse #f(0.737255 0.737255 0.737255 ))
	(instance colormaterial
		  :create :name :intensity-189
		  :diffuse #f(0.741176 0.741176 0.741176 ))
	(instance colormaterial
		  :create :name :intensity-190
		  :diffuse #f(0.745098 0.745098 0.745098 ))
	(instance colormaterial
		  :create :name :intensity-191
		  :diffuse #f(0.74902 0.74902 0.74902 ))
	(instance colormaterial
		  :create :name :intensity-192
		  :diffuse #f(0.752941 0.752941 0.752941 ))
	(instance colormaterial
		  :create :name :intensity-193
		  :diffuse #f(0.756863 0.756863 0.756863 ))
	(instance colormaterial
		  :create :name :intensity-194
		  :diffuse #f(0.760784 0.760784 0.760784 ))
	(instance colormaterial
		  :create :name :intensity-195
		  :diffuse #f(0.764706 0.764706 0.764706 ))
	(instance colormaterial
		  :create :name :intensity-196
		  :diffuse #f(0.768627 0.768627 0.768627 ))
	(instance colormaterial
		  :create :name :intensity-197
		  :diffuse #f(0.772549 0.772549 0.772549 ))
	(instance colormaterial
		  :create :name :intensity-198
		  :diffuse #f(0.776471 0.776471 0.776471 ))
	(instance colormaterial
		  :create :name :intensity-199
		  :diffuse #f(0.780392 0.780392 0.780392 ))
	(instance colormaterial
		  :create :name :intensity-200
		  :diffuse #f(0.784314 0.784314 0.784314 ))
	(instance colormaterial
		  :create :name :intensity-201
		  :diffuse #f(0.788235 0.788235 0.788235 ))
	(instance colormaterial
		  :create :name :intensity-202
		  :diffuse #f(0.792157 0.792157 0.792157 ))
	(instance colormaterial
		  :create :name :intensity-203
		  :diffuse #f(0.796078 0.796078 0.796078 ))
	(instance colormaterial
		  :create :name :intensity-204
		  :diffuse #f(0.8 0.8 0.8 ))
	(instance colormaterial
		  :create :name :intensity-205
		  :diffuse #f(0.803922 0.803922 0.803922 ))
	(instance colormaterial
		  :create :name :intensity-206
		  :diffuse #f(0.807843 0.807843 0.807843 ))
	(instance colormaterial
		  :create :name :intensity-207
		  :diffuse #f(0.811765 0.811765 0.811765 ))
	(instance colormaterial
		  :create :name :intensity-208
		  :diffuse #f(0.815686 0.815686 0.815686 ))
	(instance colormaterial
		  :create :name :intensity-209
		  :diffuse #f(0.819608 0.819608 0.819608 ))
	(instance colormaterial
		  :create :name :intensity-210
		  :diffuse #f(0.823529 0.823529 0.823529 ))
	(instance colormaterial
		  :create :name :intensity-211
		  :diffuse #f(0.827451 0.827451 0.827451 ))
	(instance colormaterial
		  :create :name :intensity-212
		  :diffuse #f(0.831373 0.831373 0.831373 ))
	(instance colormaterial
		  :create :name :intensity-213
		  :diffuse #f(0.835294 0.835294 0.835294 ))
	(instance colormaterial
		  :create :name :intensity-214
		  :diffuse #f(0.839216 0.839216 0.839216 ))
	(instance colormaterial
		  :create :name :intensity-215
		  :diffuse #f(0.843137 0.843137 0.843137 ))
	(instance colormaterial
		  :create :name :intensity-216
		  :diffuse #f(0.847059 0.847059 0.847059 ))
	(instance colormaterial
		  :create :name :intensity-217
		  :diffuse #f(0.85098 0.85098 0.85098 ))
	(instance colormaterial
		  :create :name :intensity-218
		  :diffuse #f(0.854902 0.854902 0.854902 ))
	(instance colormaterial
		  :create :name :intensity-219
		  :diffuse #f(0.858824 0.858824 0.858824 ))
	(instance colormaterial
		  :create :name :intensity-220
		  :diffuse #f(0.862745 0.862745 0.862745 ))
	(instance colormaterial
		  :create :name :intensity-221
		  :diffuse #f(0.866667 0.866667 0.866667 ))
	(instance colormaterial
		  :create :name :intensity-222
		  :diffuse #f(0.870588 0.870588 0.870588 ))
	(instance colormaterial
		  :create :name :intensity-223
		  :diffuse #f(0.87451 0.87451 0.87451 ))
	(instance colormaterial
		  :create :name :intensity-224
		  :diffuse #f(0.878431 0.878431 0.878431 ))
	(instance colormaterial
		  :create :name :intensity-225
		  :diffuse #f(0.882353 0.882353 0.882353 ))
	(instance colormaterial
		  :create :name :intensity-226
		  :diffuse #f(0.886275 0.886275 0.886275 ))
	(instance colormaterial
		  :create :name :intensity-227
		  :diffuse #f(0.890196 0.890196 0.890196 ))
	(instance colormaterial
		  :create :name :intensity-228
		  :diffuse #f(0.894118 0.894118 0.894118 ))
	(instance colormaterial
		  :create :name :intensity-229
		  :diffuse #f(0.898039 0.898039 0.898039 ))
	(instance colormaterial
		  :create :name :intensity-230
		  :diffuse #f(0.901961 0.901961 0.901961 ))
	(instance colormaterial
		  :create :name :intensity-231
		  :diffuse #f(0.905882 0.905882 0.905882 ))
	(instance colormaterial
		  :create :name :intensity-232
		  :diffuse #f(0.909804 0.909804 0.909804 ))
	(instance colormaterial
		  :create :name :intensity-233
		  :diffuse #f(0.913725 0.913725 0.913725 ))
	(instance colormaterial
		  :create :name :intensity-234
		  :diffuse #f(0.917647 0.917647 0.917647 ))
	(instance colormaterial
		  :create :name :intensity-235
		  :diffuse #f(0.921569 0.921569 0.921569 ))
	(instance colormaterial
		  :create :name :intensity-236
		  :diffuse #f(0.92549 0.92549 0.92549 ))
	(instance colormaterial
		  :create :name :intensity-237
		  :diffuse #f(0.929412 0.929412 0.929412 ))
	(instance colormaterial
		  :create :name :intensity-238
		  :diffuse #f(0.933333 0.933333 0.933333 ))
	(instance colormaterial
		  :create :name :intensity-239
		  :diffuse #f(0.937255 0.937255 0.937255 ))
	(instance colormaterial
		  :create :name :intensity-240
		  :diffuse #f(0.941176 0.941176 0.941176 ))
	(instance colormaterial
		  :create :name :intensity-241
		  :diffuse #f(0.945098 0.945098 0.945098 ))
	(instance colormaterial
		  :create :name :intensity-242
		  :diffuse #f(0.94902 0.94902 0.94902 ))
	(instance colormaterial
		  :create :name :intensity-243
		  :diffuse #f(0.952941 0.952941 0.952941 ))
	(instance colormaterial
		  :create :name :intensity-244
		  :diffuse #f(0.956863 0.956863 0.956863 ))
	(instance colormaterial
		  :create :name :intensity-245
		  :diffuse #f(0.960784 0.960784 0.960784 ))
	(instance colormaterial
		  :create :name :intensity-246
		  :diffuse #f(0.964706 0.964706 0.964706 ))
	(instance colormaterial
		  :create :name :intensity-247
		  :diffuse #f(0.968627 0.968627 0.968627 ))
	(instance colormaterial
		  :create :name :intensity-248
		  :diffuse #f(0.972549 0.972549 0.972549 ))
	(instance colormaterial
		  :create :name :intensity-249
		  :diffuse #f(0.976471 0.976471 0.976471 ))
	(instance colormaterial
		  :create :name :intensity-250
		  :diffuse #f(0.980392 0.980392 0.980392 ))
	(instance colormaterial
		  :create :name :intensity-251
		  :diffuse #f(0.984314 0.984314 0.984314 ))
	(instance colormaterial
		  :create :name :intensity-252
		  :diffuse #f(0.988235 0.988235 0.988235 ))
	(instance colormaterial
		  :create :name :intensity-253
		  :diffuse #f(0.992157 0.992157 0.992157 ))
	(instance colormaterial
		  :create :name :intensity-254
		  :diffuse #f(0.996078 0.996078 0.996078 ))
	(instance colormaterial
		  :create :name :intensity-255
		  :diffuse #f(1 1 1 ))
	))

(provide :irtglrgb "$Id$")

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;
;;; $Id$
;;;
;;; $Log$
;;; Revision 1.2  2009-02-17 02:04:48  k-okada
;;; fix typo on copyright
;;;
;;; Revision 1.1  2008/09/18 18:11:01  k-okada
;;; add irteus
;;;
;;;
