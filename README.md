## Abstract 

TACTILE (Tactical Analysis and Communication through Text Integration
with Language Engines) implements advancements in OCR techniques and
integrates a Large Language Model to extract and communicate text from
images decisively. This study intends to mitigate bias and distrust
within dual-AI technologies and develop AI-integrated software for
text analysis. This software will have commercial and military
applications to extract, interpret, and extrapolate image data. This
project is guided toward developing drone software for military
intelligence to capture and communicate text found on images into
compressed data packages discretely with a defined level of
autotomy. The commercial aspect of this software would entail the
application of automated data collection from sensor readings and
other forms of texts found in daily life. This project aims to
demonstrate the connection between the LLM, deep learning, the natural
world, and the limitations of pre-trained models. Developing
algorithms that can operate within limited computation power and
maintain accuracy and efficiency.

## Configuration
The file **.env** in the top directory must contain environment variables
which handle external dependencies:
  * The code talks to OpenAI API and it needs a valid API key
  in order to implement this functionality.
  * The code implements OCR by calling *Tesseract OCR*. Therefore, we need
  the location of the language files, so called *tessdata*. The location
  is a folder. Tesseract reads an environment variable **TESSDATA_PREFIX**
  to find the location of that folder, and therefore this variable
  needs to be set up. There are other ways to set this variable, but
  we provide a mechanism through the **.env** file. The folder is
  known to contain a file **eng.traineddata** which contains the English
  language model.
We included the file ![./env.template] which should be copied to .env
and edited to define various environment variables.

## Sources 


https://arxiv.org/abs/1904.01941
https://dojofordrones.com/
https://classic.gazebosim.org/tutorials?tut=install_ubuntu&cat=install
https://pypi.org/project/pytesseract/
https://wiki.ros.org/noetic
https://github.com/openai/openai-python
https://docs.opencv.org/4.0.1/d1/dfb/intro.html
https://wandb.ai/sayakpaul/tale-of-quantization/reports/A-Tale-of-Model-quantization-in-TF-Lite--Vmlldzo5MzQwMA
https://wiki.ros.org/ROS/Introduction
https://github.com/tesseract-ocr/tessdoc
https://dronekit-python.readthedocs.io/en/latest/Drone
https://ardupilot.org/dev/index.html
https://arxiv.org/abs/1704.03155
