
.section	".text" , "ax"
.align	4
.globl	_training
.type   _training, @function
_training:
    /*****************************
     * START
     *****************************/

    addi 1024, r9 r19


    /*****************************
     * END
     *****************************/
    br _main
.size		_training, .-_training
