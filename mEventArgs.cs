using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Microsoft.Samples.Kinect.BodyBasics
{
    public class mEventArgs : EventArgs
    {
        public int Args { get; set; }

        public mEventArgs(int args)
        {
            Args = args;
        }
    }
}
