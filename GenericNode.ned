//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
// 
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
// 
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see http://www.gnu.org/licenses/.
// 

//
// A node showing a 3D model with position, transmission range,...
//
simple GenericNode
{
    parameters:
        string modelURL;                             // the URL/filename of the 3D model to be used for the node
                                                     // (you can use osg pseudo filters like: modelname.osgb.3.scale.0,0,45.rot scaleX3, rotate 45 deg around Z)
        string modelColor = default("");             // colorizes the 3D model used for the node
        // decorations and annotations
        string labelColor = default("#ffff00ff");    // the color of the model label in hex RRGGBBAA format or "" to turn off labels
        string label2Color = default("#cccc00ff");   // the color of the second model label in hex RRGGBBAA format or "" to turn off labels
        double txRange @unit("m") = default(200m);   // the transmission range of the mobile node's radio
        bool showTxRange = default(false);           // whether to show the transmission range around the nodes
        string rangeColor = default("#ff000040");    // the color of the range indicator in hex RRGGBBAA format
        // simulation
        double timeStep @unit("s") = default(33ms);  // the time granularity of movement calculation
        double startTime @unit("s") = default(0s);   // time when the movement starts
}
