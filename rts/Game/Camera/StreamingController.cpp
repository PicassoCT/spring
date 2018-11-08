#include "StreamingController.h"	


StreamingController::StreamingController(boost::asio::ip::address_v4 ipAdress, unsigned int port){
	//open bit stream
	 socket_.async_receive_from(
     boost::asio::buffer(recv_buffer_), remote_endpoint_,
     boost::bind(&udp_server::handle_receive, this,
     boost::asio::placeholders::error,
     boost::asio::placeholders::bytes_transferred));
	
       try
	  {
	    boost::asio::io_service io_service;
	    udp_server server(io_service);
	    io_service.run();
	  }
	  catch (std::exception& e)
	  {
	    std::cerr << e.what() << std::endl;
	  }
	
}

StreamingController::initStream(void) {
	int rv = WelsCreateSVCEncoder (&encoder_);
	ASSERT_EQ (0, rv);
	ASSERT_TRUE (encoder_ != NULL);

	memset (&param, 0, sizeof (SEncParamBase));
	param.iUsageType = usageType;
	param.fMaxFrameRate = frameRate;
	param.iPicWidth = width;
	param.iPicHeight = height;
	param.iTargetBitrate = 5000000;
	encoder_->Initialize (&param);

	int videoFormat = videoFormatI420;
	encoder_->SetOption (ENCODER_OPTION_DATAFORMAT, &videoFormat);

	//fill in instant param content
	encoder_->SetOption (ENCODER_OPTION_SVC_ENCODE_PARAM_BASE, &param);
}

StreamingController::void handle_receive(const boost::system::error_code& error,
      std::size_t /*bytes_transferred*/){

}

StreamingController::handle_send(
								boost::shared_ptr<std::string> /*message*/,
      							const boost::system::error_code& /*error*/,
      							std::size_t /*bytes_transferred*/ ){
	
	int frameSize = width * height * 3 / 2;
	BufferedData buf;
	buf.SetLength (frameSize);
	
	ASSERT_TRUE (buf.Length() == (size_t)frameSize);
	SFrameBSInfo info;
	memset (&info, 0, sizeof (SFrameBSInfo));
	SSourcePicture pic;
	memset (&pic, 0, sizeof (SsourcePicture));
	pic.iPicWidth = width;
	pic.iPicHeight = height;
	pic.iColorFormat = videoFormatI420;
	pic.iStride[0] = pic.iPicWidth;
	pic.iStride[1] = pic.iStride[2] = pic.iPicWidth >> 1;
	pic.pData[0] = buf.data();
	pic.pData[1] = pic.pData[0] + width * height;
	pic.pData[2] = pic.pData[1] + (width * height >> 2);
	
	
	   //prepare input data
	   rv = encoder_->EncodeFrame (&pic, &info);
	   ASSERT_TRUE (rv == cmResultSuccess);
	   if (info.eFrameType != videoFrameTypeSkip && cbk != 	NULL) {	

		 if (!ec) {
					{	
					stream <<  imageBufferToByteStream();  
					} 
					catch (std::exception& e) {
					 std::cerr << e.what() << std::endl;
					 }
				 }
		}
}


StreamingController::dismantle(){
	
	if (encoder_) { 
		encoder_->Uninitialize(); 
		WelsDestroySVCEncoder (encoder_);
	 }

}
