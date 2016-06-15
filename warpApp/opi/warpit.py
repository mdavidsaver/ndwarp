from org.csstudio.opibuilder.scriptUtil import PVUtil, ConsoleUtil

# pvs
# 0 - "Selected" column of pixel VTable
#   =columnOf('loc://$(DID)pixel', "Selected")
pv_sel = pvs[0]
# 1 - "X" column of pixel VTable
#   =columnOf('loc://$(DID)pixel', "X")
pv_px = pvs[1]
# 2 - "Y" column of pixel VTable
#   =columnOf('loc://$(DID)pixel'   , "Y")
pv_py = pvs[2]
# 3 - Selection mode
#   loc://$(DID)selmode(0)
pv_mode = pvs[3]
# 4 - saved point X
#   loc://$(DID)SaveX
pv_savex = pvs[4]
# 5 - saved point Y
#   loc://$(DID)SaveXY
pv_savey = pvs[5]
# 6 - Image height
#   $(P)image1:ArraySize1_RBV
pv_height = pvs[6]
# 7 - Destination for center X
#   $(WARP)CenterX
pv_cx = pvs[7]
# 8 - Destination for center Y
#   $(WARP)CenterY
pv_cy = pvs[8]
# 9 - Destination for Length
#   $(WARP)LPX_CSET
pv_len = pvs[9]

isdown = PVUtil.getDouble(pv_sel)
if isdown>0.5:
	next_mode = 0

	# widget coordinates to image coordinates
	X, Y = PVUtil.getDouble(pv_px), PVUtil.getDouble(pv_py)
	H = PVUtil.getDouble(pv_height) 
	X -= 4  # empirically necessary to align w/ cursor ...
	Y = H-Y

	mode = PVUtil.getDouble(pv_mode)

	if mode==1:
		# select and store center

		# don't use 'long' with setValue as jython maps this to BigInteger, which pvManager doesn't understand
		pv_cx.setValue(X)
		pv_cy.setValue(Y)

	elif mode==2:
		# select point 1

		pv_savex.setValue(X)
		pv_savey.setValue(Y)
		next_mode = 3

	elif mode==3:
		# select point 2 and store length

		PX, PY = PVUtil.getDouble(pv_savex), PVUtil.getDouble(pv_savey)

		L = max(abs(PX-X), abs(PY-Y))
		pv_len.setValue(L)

	pv_mode.setValue(next_mode)
