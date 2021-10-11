package frc.robot.base;

// ===============================================================================
//      class clsNetTblEntryInfo
//          helper class to assist in reading and writing network table values
//
// ----------------------- revision history --------------------------------------
//
// ===============================================================================

// unused import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;


// --------to optimize network table I/O
public class clsNetTblEntryInfo {

    private String strName = "";
    private boolean boolEntryFound = false;
    private boolean boolNetTableSet = false;
    private NetworkTableEntry objNTEntry;
    private NetworkTableInstance objNetTbl;

    public clsNetTblEntryInfo( String name ) {
      // --------should check for blank!
      strName = name;
    }

    //--------set the network table instance
    public void SetTable( NetworkTableInstance inst ) {
      objNetTbl = inst;
      boolNetTableSet = true;
    }

    //--------set the specific entry for this string name
    private void CheckSetEntry(  ) {
      if ( !boolEntryFound ) {
        objNTEntry = this.objNetTbl.getEntry( this.strName );
        boolEntryFound = true;
      }
    }
    
    //--------read a double
    public double GetDouble( double default_value ) {
      double rtnValue = default_value;

      if ( boolNetTableSet) {
        CheckSetEntry();
        rtnValue = this.objNTEntry.getDouble( default_value );
      }
      return rtnValue;
    }

    //--------write a boolean
    public void WriteBoolean( boolean value ) {
      if ( boolNetTableSet) {
        CheckSetEntry();
        this.objNTEntry.forceSetBoolean(value);
        // debug  System.out.println(this.strName + ": " + value);
      }        
    }

    //--------write a double
    public void WriteDouble( double value ) {
      if ( boolNetTableSet) {
        CheckSetEntry();
        this.objNTEntry.forceSetDouble(value);
        // debug  System.out.println(this.strName + ": " + value);
      }        
    }

    //--------set persistent.  the robot will keep track of the value
    //--------and save it in a file on the robot.  This is supposed
    //--------to allow value to "persistent" between robot reboots.
    public void SetPersistent( double initialValue ) {
      if ( boolNetTableSet) {
        CheckSetEntry();
        //--------setting persistence is a one time thing. Only write
        //--------the default value if the entry is not persistent yet!
        if ( !this.objNTEntry.isPersistent() ) {
          this.WriteDouble(initialValue);
          this.objNTEntry.setPersistent();
        }
      }
    }

  }   // end cls
