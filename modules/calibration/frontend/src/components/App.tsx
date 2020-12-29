import * as React from "react";
import { Navbar, Nav } from "react-bootstrap";
import { Switch, Route, Link } from "react-router-dom";
import { State } from "@farm-ng/frontend/src/components/State";
import { Video } from "@farm-ng/frontend/src/components/Video";
import { Root as Scope } from "@farm-ng/frontend/src/components/scope/Root";
import { Programs } from "@farm-ng/frontend/src/components/Programs";
import { Blobstore } from "@farm-ng/frontend/src/components/Blobstore";
import styles from "./App.module.scss";

export const App: React.FC = () => {
  return (
    <div className={styles.app}>
      <Navbar
        collapseOnSelect
        expand="md"
        bg="dark"
        variant="dark"
        className={styles.navbar}
      >
        <Navbar.Brand as={Link} to="/">
          farm-ng calibration
        </Navbar.Brand>
        <Navbar.Toggle aria-controls="responsive-navbar-nav" />
        <Navbar.Collapse id="responsive-navbar-nav">
          <Nav>
            <Nav.Link as={Link} to="/state">
              State
            </Nav.Link>
            <Nav.Link as={Link} to="/video">
              Video
            </Nav.Link>
            <Nav.Link as={Link} to="/programs">
              Programs
            </Nav.Link>
            <Nav.Link as={Link} to="/scope">
              Scope
            </Nav.Link>
            <Nav.Link as={Link} to="/blobs">
              Blobs
            </Nav.Link>
          </Nav>
        </Navbar.Collapse>
      </Navbar>
      <Switch>
        <Route exact path="/" component={Programs} />
        <Route exact path="/state" component={State} />
        <Route exact path="/video" component={Video} />
        <Route exact path="/programs" component={Programs} />
        <Route exact path="/scope" component={Scope} />
        <Route path={["/blobs/:blobPath+", "/blobs"]} component={Blobstore} />
        <Route render={() => <p>Not found</p>} />
      </Switch>
    </div>
  );
};
