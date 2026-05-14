# Validate and launch the ADI HDL ZedBoard/FMCOMMS3 reference flow.
# Usage:
#   vivado -mode batch -source fpga/adi_base/create_zed_fmcomms3_project.tcl -tclargs /path/to/analogdevicesinc/hdl
#
# Expected output:
#   ADI-generated fmcomms2_zed Vivado project under the supplied HDL tree.
#
# Notes:
#   AD-FMCOMMS2/3/4 share the ADI fmcomms2 HDL project family. This script
#   intentionally delegates AD9361 pin, timing, DMA, and Linux integration to
#   the ADI reference design, then the local ssb_toa IP can be added as an
#   extra repo path in the generated project.

if {$argc < 1} {
  error "Pass the ADI HDL repository path as the first tclarg"
}

set adi_hdl_dir [file normalize [lindex $argv 0]]
set project_dir [file join $adi_hdl_dir projects fmcomms2 zed]
set project_tcl [file join $project_dir system_project.tcl]

if {![file exists $project_tcl]} {
  error "Could not find ADI fmcomms2_zed project script: $project_tcl"
}

puts "ADI HDL directory: $adi_hdl_dir"
puts "ADI project script: $project_tcl"
cd $project_dir
source $project_tcl

puts "ADI fmcomms2_zed project creation finished"
