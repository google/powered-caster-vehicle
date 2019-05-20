/*======================================================================
Copyright 2019 Google LLC

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    https://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
======================================================================*/

/**********************************************/
/** THIS IS A STARTING POINT FOR CODE THAT   **/
/** COULD BE CREATED FOR THE HWInterface     **/
/** OR REPLACE THIS FILE WITH YOUR INTERFACE **/
/**********************************************/


class HWInterface
{
public:

    HandleGet(void) { return gHWI; }

    void RawDAC(unsigned short nAxis, long lCounts)
    void EncoderLatch(void)
    void SetEncoderCounts(unsigned short nAxis, long lCounts)
    void SelectIndexAxis(unsigned char byAxis, unsigned char byPol)
    short IndexPulse(void)
    unsigned short IndexPulseLatch(void)
    void EncReadAll(STG_LONGBYTE *lbEnc)

    // A HANDY HW REGISTER TO STORE THE STATE OF CALIBRATION
    // THE USER WILL NEED TO FIND A SUITABLE METHOD THAT
    // RESETS ITSELF WHEN POWER IS CYCLED
    int IsCalib(bool state)
    int IsCalib(void)

private:

    HWInterface(void)
    ~HWInterface(void)

    bool IsValid()
}
