utcomponents
============
This is the utcomponents Ubitrack submodule.

Usage
-----
In order to use it, you have to clone the buildenvironment, change to the ubitrack directory and add the utcomponents by executing:

    git submodule add https://github.com/schwoere/utcomponents.git modules/utcomponents

Description
----------
The utcomponents contains contains basic dataflow modules. These cover most common tracking and registration cases.

Dependencies
----------
In addition, this module has to following submodule dependencies which have to be added for successful building:

<table>

  <tr>
    <th>Dependency</th><th>Dependent Components</th><th>optional Dependency</th>
  </tr>
  <tr>
    <td>utdataflow</td><td>utComponents, utIOComponents</td><td>no</td>
  </tr>
</table>
