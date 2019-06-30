# morningstar-rs

Gather stats from, and control morningstar solar charge
controlers. Currently only the Prostar MPPT is supported, because
that's the only one I have physical access to.

see document http://support.morningstarcorp.com/wp-content/uploads/2015/12/PSMPPT_public-MODBUS-doc_v04.pdf for details on the modbus interface.

Programming the charge controller is now supported. No checks are done
to the parameters you set, and I cannot be responsible for the outcome
of an improperly programmed controler, you could set fire to your
house, destroy your batteries, destroy your charge controllers, or
summon lightning from the angry gods. All that said, I program my Prostar
MPPT40M using this code, and it works just fine :-)
