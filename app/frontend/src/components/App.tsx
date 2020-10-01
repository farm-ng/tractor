import * as React from "react";
import { Navbar, Nav } from "react-bootstrap";
import { Switch, Route, Link } from "react-router-dom";
import { Overview } from "./Overview";
import { State } from "./State";
import { Video } from "./Video";
import { Map } from "./Map";
import { Root as Scope } from "./scope/Root";
import styles from "./App.module.scss";
import { Programs } from "./Programs";

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
          farm-ng
        </Navbar.Brand>
        <Navbar.Toggle aria-controls="responsive-navbar-nav" />
        <Navbar.Collapse id="responsive-navbar-nav">
          <Nav>
            <Nav.Link as={Link} to="/overview">
              Overview
            </Nav.Link>
            <Nav.Link as={Link} to="/state">
              State
            </Nav.Link>
            <Nav.Link as={Link} to="/video">
              Video
            </Nav.Link>
            <Nav.Link as={Link} to="/map">
              Map
            </Nav.Link>
            <Nav.Link as={Link} to="/programs">
              Programs
            </Nav.Link>
            <Nav.Link as={Link} to="/scope">
              Scope
            </Nav.Link>
          </Nav>
        </Navbar.Collapse>
      </Navbar>
      <Switch>
        <Route exact path="/" component={Overview} />
        <Route exact path="/overview" component={Overview} />
        <Route exact path="/state" component={State} />
        <Route exact path="/video" component={Video} />
        <Route exact path="/map" component={Map} />
        <Route exact path="/programs" component={Programs} />
        <Route exact path="/scope" component={Scope} />
        <Route render={() => <p>Not found</p>} />
      </Switch>
    </div>
  );
};
