MEMORY
{
  /* 256 Kilobytes of non-volatile read-only Flash memory for our program code */
  FLASH : ORIGIN = 0x08000000, LENGTH = 256K
  
  /* 64 Kilobytes of volatile RAM for variables and runtime stacks */
  RAM   : ORIGIN = 0x20000000, LENGTH = 64K
}